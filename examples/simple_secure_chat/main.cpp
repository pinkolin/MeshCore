#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>
#include "utils.h"

// Forward declaration of base64 functions (defined in BaseChatMesh.cpp via base64.hpp)
unsigned int encode_base64(const unsigned char input[], unsigned int input_length, unsigned char output[]);
unsigned int decode_base64(const unsigned char input[], unsigned int input_length, unsigned char output[]);

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "v3 (build: 09 Oct 2025)"

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#include <helpers/BaseChatMesh.h>

// Note: MAX_GROUP_CHANNELS is defined in platformio.ini (e.g., 4)
// We reserve 1 slot for built-in Public channel, leaving MAX_GROUP_CHANNELS-1 for user channels

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="
#define  TEST_GROUP_PSK    "MDAwMDAwMDAwMDAwMDAwMA=="

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/* -------------------------------------------------------------------------------------- */

struct UserChannel {
  char name[32];          // channel name (or hashtag like #mychannel)
  char key_hex[64];       // PSK as hex string (32 bytes = 64 hex chars), empty for hashtag channels
  bool muted;             // mute notifications from this channel
  bool active;            // is this slot used?
};

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t tx_power_dbm;
  uint8_t sf;           // spread factor
  uint8_t cr;           // coding rate
  float bw;             // bandwidth
  bool mute_adverts;    // mute advert notifications
  UserChannel channels[MAX_GROUP_CHANNELS - 1];  // user-defined channels
  int selected_channel_idx;  // currently selected channel for 'ch' command (-1 = none, 0 = public)
};

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  uint32_t expected_ack_crc;
  ChannelDetails* active_channels[MAX_GROUP_CHANNELS];  // +1 for built-in Public channel
  bool channel_muted[MAX_GROUP_CHANNELS];
  unsigned long last_msg_sent;
  ContactInfo* curr_recipient;
  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  // Helper class for collecting matching contacts
  class AutocompleteVisitor : public ContactVisitor {
  public:
    const char* prefix;
    int prefix_len;
    char matching_names[MAX_CONTACTS][32];
    int match_count;
    
    AutocompleteVisitor() : match_count(0), prefix(NULL), prefix_len(0) {}
    
    void reset(const char* search_prefix) {
      prefix = search_prefix;
      prefix_len = strlen(search_prefix);
      match_count = 0;
    }
    
    void onContactVisit(const ContactInfo& contact) override {
      if (match_count >= MAX_CONTACTS) return;
      
      // Case-insensitive prefix match
      bool match = true;
      for (int j = 0; j < prefix_len && contact.name[j] != 0; j++) {
        char c1 = contact.name[j];
        char c2 = prefix[j];
        // Convert to lowercase for comparison
        if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
        if (c2 >= 'A' && c2 <= 'Z') c2 += 32;
        if (c1 != c2) {
          match = false;
          break;
        }
      }
      
      if (match && contact.name[0] != 0) {
        strncpy(matching_names[match_count], contact.name, 32);
        matching_names[match_count][31] = 0;
        match_count++;
      }
    }
  };
  
  AutocompleteVisitor autocomplete_visitor;

  // Helper function to redraw the prompt with current command buffer
  void redrawPrompt() {
    Serial.print("\r> ");
    Serial.print(command);  // print current command buffer
  }

  // Handle tab completion for "to" command
  void handleTabCompletion(int& len) {
    // Check if we're in a "to " command
    if (len >= 3 && memcmp(command, "to ", 3) == 0) {
      const char* prefix = &command[3];
      
      // Use the same mechanism as "list" command to scan contacts
      autocomplete_visitor.reset(prefix);
      scanRecentContacts(0, &autocomplete_visitor);  // 0 = scan all contacts
      
      if (autocomplete_visitor.match_count == 1) {
        // Single match - autocomplete it
        snprintf(&command[3], sizeof(command) - 3, "%s", autocomplete_visitor.matching_names[0]);
        len = 3 + strlen(autocomplete_visitor.matching_names[0]);
        command[len] = 0;
        
        // Redraw from beginning
        Serial.print("\r> ");
        Serial.print(command);
      } else if (autocomplete_visitor.match_count > 1) {
        // Multiple matches - show them
        Serial.println();
        Serial.println("Matches:");
        for (int i = 0; i < autocomplete_visitor.match_count; i++) {
          Serial.print("   ");
          Serial.println(autocomplete_visitor.matching_names[i]);
        }
        
        // Redraw prompt with current buffer
        redrawPrompt();
      } else {
        // No matches - just beep or do nothing
        Serial.print('\a');  // bell character (optional)
      }
    }
    // Check if we're in a channel-related command: chsel, mute ch, unmute ch, del ch
    else if ((len >= 6 && memcmp(command, "chsel ", 6) == 0) ||
             (len >= 8 && memcmp(command, "mute ch ", 8) == 0) ||
             (len >= 10 && memcmp(command, "unmute ch ", 10) == 0) ||
             (len >= 7 && memcmp(command, "del ch ", 7) == 0)) {
      
      // Determine the prefix position
      const char* prefix = nullptr;
      int prefix_offset = 0;
      if (memcmp(command, "chsel ", 6) == 0) {
        prefix = &command[6];
        prefix_offset = 6;
      } else if (memcmp(command, "mute ch ", 8) == 0) {
        prefix = &command[8];
        prefix_offset = 8;
      } else if (memcmp(command, "unmute ch ", 10) == 0) {
        prefix = &command[10];
        prefix_offset = 10;
      } else if (memcmp(command, "del ch ", 7) == 0) {
        prefix = &command[7];
        prefix_offset = 7;
      }
      
      // Collect matching channels
      char matching_channels[MAX_GROUP_CHANNELS][32];
      int match_count = 0;
      int prefix_len = strlen(prefix);
      
      // Check Public channel
      if (prefix_len == 0 || strncasecmp("Public", prefix, prefix_len) == 0) {
        strcpy(matching_channels[match_count++], "Public");
      }
      
      // Check user channels
      for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
        if (_prefs.channels[i].active) {
          if (prefix_len == 0 || strncasecmp(_prefs.channels[i].name, prefix, prefix_len) == 0) {
            strcpy(matching_channels[match_count++], _prefs.channels[i].name);
          }
        }
      }
      
      if (match_count == 1) {
        // Single match - autocomplete it
        snprintf(&command[prefix_offset], sizeof(command) - prefix_offset, "%s", matching_channels[0]);
        len = prefix_offset + strlen(matching_channels[0]);
        command[len] = 0;
        
        // Redraw from beginning
        Serial.print("\r> ");
        Serial.print(command);
      } else if (match_count > 1) {
        // Multiple matches - show them
        Serial.println();
        Serial.println("Matches:");
        for (int i = 0; i < match_count; i++) {
          Serial.print("   ");
          Serial.println(matching_channels[i]);
        }
        
        // Redraw prompt with current buffer
        redrawPrompt();
      } else {
        // No matches
        Serial.print('\a');  // bell character
      }
    }
  }

  // Helper: Find channel index in active_channels[] by name (-1 if not found, 0 = public, 1+ = user channels)
  int findChannelByName(const char* name) {
    if (strcasecmp(name, "public") == 0 || strcasecmp(name, "pub") == 0) {
      return 0;  // built-in Public channel
    }
    
    // Find the prefs channel with matching name and compute its base64 key
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (_prefs.channels[i].active && strcasecmp(_prefs.channels[i].name, name) == 0) {
        // Found the prefs entry - now find its position in active_channels[]
        // We need to count how many active prefs channels come before this one
        int active_idx = 1;  // Start at 1 because 0 is Public
        for (int j = 0; j < MAX_GROUP_CHANNELS - 1; j++) {
          if (_prefs.channels[j].active) {
            if (j == i) {
              // This is our channel
              return active_idx;
            }
            active_idx++;
          }
        }
      }
    }
    return -1;
  }

  // Helper: Get channel name by active_channels[] index
  const char* getChannelName(int idx) {
    if (idx == 0) return "Public";
    if (idx > 0 && idx < MAX_GROUP_CHANNELS && active_channels[idx] != nullptr) {
      // Find which prefs entry corresponds to this active_channels index
      // Count through active prefs channels to find the idx-th one
      int active_count = 0;
      for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
        if (_prefs.channels[i].active) {
          active_count++;
          if (active_count == idx) {
            return _prefs.channels[i].name;
          }
        }
      }
    }
    return nullptr;
  }

  // Helper: Get channel name from prefs by index (works even before initChannels)
  const char* getChannelNameFromPrefs(int idx) {
    if (idx == 0) return "Public";
    if (idx > 0 && idx < MAX_GROUP_CHANNELS) {
      // Count through active prefs channels to find the idx-th one
      int active_count = 0;
      for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
        if (_prefs.channels[i].active) {
          active_count++;
          if (active_count == idx) {
            return _prefs.channels[i].name;
          }
        }
      }
    }
    return nullptr;
  }

  // Helper: Add or update user channel
  bool setUserChannel(const char* name, const char* key_hex) {
    // Check if channel already exists
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (_prefs.channels[i].active && strcasecmp(_prefs.channels[i].name, name) == 0) {
        // Update existing
        strncpy(_prefs.channels[i].key_hex, key_hex, sizeof(_prefs.channels[i].key_hex) - 1);
        _prefs.channels[i].key_hex[sizeof(_prefs.channels[i].key_hex) - 1] = 0;
        return true;
      }
    }
    
    // Find empty slot
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (!_prefs.channels[i].active) {
        strncpy(_prefs.channels[i].name, name, sizeof(_prefs.channels[i].name) - 1);
        _prefs.channels[i].name[sizeof(_prefs.channels[i].name) - 1] = 0;
        strncpy(_prefs.channels[i].key_hex, key_hex, sizeof(_prefs.channels[i].key_hex) - 1);
        _prefs.channels[i].key_hex[sizeof(_prefs.channels[i].key_hex) - 1] = 0;
        _prefs.channels[i].active = true;
        _prefs.channels[i].muted = false;
        return true;
      }
    }
    return false;  // no space
  }

  // Helper: Remove user channel
  bool removeUserChannel(const char* name) {
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (_prefs.channels[i].active && strcasecmp(_prefs.channels[i].name, name) == 0) {
        _prefs.channels[i].active = false;
        return true;
      }
    }
    return false;
  }

  // Helper: Convert hex string to base64 using base64 library (supports 32 or 64 hex chars)
  void hexToBase64(char* dest, const char* hex_str, int hex_len) {
    uint8_t bytes[32];
    int byte_len = hex_len / 2;
    mesh::Utils::fromHex(bytes, byte_len, hex_str);
    unsigned int b64_len = encode_base64(bytes, byte_len, (unsigned char*)dest);
    dest[b64_len] = 0;
  }

  // Helper: Initialize channels from prefs
  void initChannels() {
    // Clear all channel pointers
    for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
      active_channels[i] = nullptr;
      channel_muted[i] = false;
    }
    
    // Add built-in Public channel (always index 0)
    active_channels[0] = addChannel("Public", PUBLIC_GROUP_PSK);
    channel_muted[0] = false;
    
    // Add user channels from prefs - use separate counter for active_channels index
    int active_idx = 1;  // Start at 1, because 0 is Public
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (_prefs.channels[i].active && active_idx < MAX_GROUP_CHANNELS) {
        ChannelDetails* ch = nullptr;
        if (_prefs.channels[i].name[0] == '#') {
          // Hashtag channel - hash the name first, use first 16 bytes, then base64 encode
          // This matches the mobile app behavior: first 16 bytes of SHA256(name) is used as the secret
          uint8_t hash[32];
          mesh::Utils::sha256(hash, sizeof(hash), (const uint8_t*)_prefs.channels[i].name, strlen(_prefs.channels[i].name));
          
          char key_b64[64];
          unsigned int b64_len = encode_base64(hash, 16, (unsigned char*)key_b64);  // Use only first 16 bytes!
          key_b64[b64_len] = 0;
          ch = addChannel(_prefs.channels[i].name, key_b64);
        } else if (_prefs.channels[i].key_hex[0] != 0) {
          // Regular channel with hex key - convert hex to base64 for addChannel
          int key_len = strlen(_prefs.channels[i].key_hex);
          if (key_len == 32 || key_len == 64) {
            char key_b64[64];
            hexToBase64(key_b64, _prefs.channels[i].key_hex, key_len);
            ch = addChannel(_prefs.channels[i].name, key_b64);
          }
        }
        
        if (ch != nullptr) {
          active_channels[active_idx] = ch;
          channel_muted[active_idx] = _prefs.channels[i].muted;
          active_idx++;
        }
      }
    }
  }

  const char* getTypeName(uint8_t type) const {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??";  // unknown
  }

  void loadContacts() {
    if (_fs->exists("/contacts")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/contacts", "r");
    #else
      File file = _fs->open("/contacts");
    #endif
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;   // not yet supported

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
#if defined(NRF52_PLATFORM)
    _fs->remove("/contacts");
    File file = _fs->open("/contacts", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File file = _fs->open("/contacts", "w");
#else
    File file = _fs->open("/contacts", "w", true);
#endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (!success) break;  // write failed
      }
      file.close();
    }
  }

  void setClock(uint32_t timestamp) {
    uint32_t curr = getRTCClock()->getCurrentTime();
    if (timestamp > curr) {
      getRTCClock()->setCurrentTime(timestamp);
      Serial.println("   (OK - clock set!)");
    } else {
      Serial.println("   (ERR: clock cannot go backwards)");
    }
  }

  void importCard(const char* command) {
    while (*command == ' ') command++;   // skip leading spaces
    if (memcmp(command, "meshcore://", 11) == 0) {
      command += 11;  // skip the prefix
      char *ep = strchr(command, 0);  // find end of string
      while (ep > command) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;  // found tail end of card
        *ep = 0;  // remove trailing spaces and other junk
      }
      int len = strlen(command);
      if (len % 2 == 0) {
        len >>= 1;  // halve, for num bytes
        if (mesh::Utils::fromHex(tmp_buf, len, command)) {
          importContact(tmp_buf, len);
          return;
        }
      }
    }
    Serial.println("   error: invalid format");
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    return 0;  // disable rxdelay
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    if (!_prefs.mute_adverts) {
      // TODO: if not in favs,  prompt to add as fav(?)
      Serial.print("\r\n");
      Serial.printf("ADVERT from -> %s", contact.name);
      Serial.printf(" | type: %s", getTypeName(contact.type));
      Serial.print(" | public key: "); mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE); Serial.println();
      redrawPrompt();
    }

    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    Serial.print("\r\n");
    Serial.printf("PATH to: %s, path_len=%d\n", contact.name, (int32_t) contact.out_path_len);
    redrawPrompt();
    saveContacts();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      Serial.print("\r\n");
      Serial.printf("   Got ACK! (round trip: %d millis)\n", _ms->getMillis() - last_msg_sent);
      redrawPrompt();
      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      return curr_recipient;  // Return the recipient to cancel timeout
    }
    return NULL;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // Create a mutable copy of the text to remove diacritics
    char text_copy[256];
    strncpy(text_copy, text, sizeof(text_copy) - 1);
    text_copy[sizeof(text_copy) - 1] = 0;
    removeDiacritics(text_copy);
    
    Serial.print("\r\n");  // carriage return + newline to separate from any input
    Serial.printf("(%s) MSG -> from %s | ", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", from.name);
    Serial.printf(": %s\n", text_copy);
    redrawPrompt();

    if (strcmp(text, "clock sync") == 0) {  // special text command
      setClock(sender_timestamp + 1);
    }
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
  }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {
    // Find which channel this is
    const char* channel_name = "UNKNOWN";
    bool is_muted = false;
    
    for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
      if (active_channels[i] && memcmp(channel.hash, active_channels[i]->channel.hash, sizeof(channel.hash)) == 0) {
        channel_name = getChannelName(i);
        is_muted = channel_muted[i];
        break;
      }
    }
    
    if (is_muted) {
      return;  // Channel is muted, don't display
    }
    
    // Create a mutable copy of the text to remove diacritics
    char text_copy[256];
    strncpy(text_copy, text, sizeof(text_copy) - 1);
    text_copy[sizeof(text_copy) - 1] = 0;
    removeDiacritics(text_copy);
    
    Serial.print("\r\n");  // carriage return + newline to separate from any input
    
    if (pkt->isRouteDirect()) {
      Serial.printf("[%s] DIRECT | %s\n", channel_name, text_copy);
    } else {
      Serial.printf("[%s] FLOOD (hops %d) | %s\n", channel_name, pkt->path_len, text_copy);
    }
    redrawPrompt();
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 2.0;    // one third
    strcpy(_prefs.node_name, "NONAME");
    _prefs.freq = LORA_FREQ;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;
    _prefs.bw = LORA_BW;
    _prefs.mute_adverts = false;  // by default, show adverts
    _prefs.selected_channel_idx = 0;  // default to Public channel
    
    // Initialize channels array
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      _prefs.channels[i].active = false;
      _prefs.channels[i].name[0] = 0;
      _prefs.channels[i].key_hex[0] = 0;
      _prefs.channels[i].muted = false;
    }

    command[0] = 0;
    curr_recipient = NULL;
  }

  float getFreqPref() const { return _prefs.freq; }
  uint8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }
  uint8_t getSFPref() const { return _prefs.sf; }
  uint8_t getCRPref() const { return _prefs.cr; }
  float getBWPref() const { return _prefs.bw; }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {  // legacy: node_name was from identity file
      // Need way to get some entropy to seed RNG
      Serial.println("Press ENTER to generate key:");
      char c = 0;
      while (c != '\n') {   // wait for ENTER to be pressed
        if (Serial.available()) c = Serial.read();
      }
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/node_prefs", "r");
    #else
      File file = _fs->open("/node_prefs");
    #endif
      if (file) {
        file.read((uint8_t *) &_prefs, sizeof(_prefs));
        file.close();
      }
    }

    loadContacts();
    initChannels();  // Initialize all channels from prefs
  }

  void checkPublicChannel() {
    if (active_channels[0] == NULL) {
      Serial.println("ERROR: Failed to add Public channel!");
      Serial.println("This usually means base64 decoding failed or PSK has wrong length.");
      Serial.print("PSK used: "); Serial.println(PUBLIC_GROUP_PSK);
    } else {
      Serial.println("Public channel initialized successfully!");
    }
    
    // Show user channels
    int user_ch_count = 0;
    for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
      if (_prefs.channels[i].active) user_ch_count++;
    }
    if (user_ch_count > 0) {
      Serial.printf("%d user channel(s) loaded\n", user_ch_count);
    }
  }

  void savePrefs() {
#if defined(NRF52_PLATFORM)
    _fs->remove("/node_prefs");
    File file = _fs->open("/node_prefs", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File file = _fs->open("/node_prefs", "w");
#else
    File file = _fs->open("/node_prefs", "w", true);
#endif
    if (file) {
      file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void showWelcome() {
    delay(100);  // Give serial monitor time to connect
    Serial.println();
    Serial.println(" _      ____    _____ _____ ____  _     ");
    Serial.println("/ \\__/|/   _\\  /__ __Y  __//  __\\/ \\__/|");
    Serial.println("| |\\/|||  /      / \\ |  \\  |  \\/|| |\\/||");
    Serial.println("| |  |||  \\__    | | |  /_ |    /| |  ||");
    Serial.println("\\_/  \\|\\____/    \\_/ \\____\\\\_/\\_\\\\_/  \\|");
    Serial.println("   ===== MeshCore Chat Terminal =====");
    Serial.println();
    Serial.printf("WELCOME  %s\n\r", _prefs.node_name);
    mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE);
    Serial.println();
    Serial.println("(enter 'help' for basic commands)");
    Serial.println();
    Serial.print("\r> ");  // initial prompt
  }

  void sendSelfAdvert(int delay_millis) {
    auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    Serial.printf("   %s - ", contact.name);
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;  // skip leading spaces

    if (memcmp(command, "send ", 5) == 0) {
      if (curr_recipient) {
        const char *text = &command[5];
        uint32_t est_timeout;

        int result = sendMessage(*curr_recipient, getRTCClock()->getCurrentTime(), 0, text, expected_ack_crc, est_timeout);
        if (result == MSG_SEND_FAILED) {
          Serial.println("   ERROR: unable to send.");
        } else {
          last_msg_sent = _ms->getMillis();
          Serial.printf("   (message sent - %s)\n", result == MSG_SEND_SENT_FLOOD ? "FLOOD" : "DIRECT");
        }
      } else {
        Serial.println("   ERROR: no recipient selected (use 'to' cmd).");
      }
    } else if (memcmp(command, "ch ", 3) == 0) {  // send to selected channel
      if (_prefs.selected_channel_idx < 0) {
        Serial.println("   ERROR: No channel selected (use 'chsel <name>')");
        return;
      }
      
      ChannelDetails* ch = active_channels[_prefs.selected_channel_idx];
      if (ch == NULL) {
        Serial.println("   ERROR: Selected channel not initialized!");
        return;
      }
      
      uint8_t temp[5+MAX_TEXT_LEN+32];
      uint32_t timestamp = getRTCClock()->getCurrentTime();
      memcpy(temp, &timestamp, 4);
      temp[4] = 0;

      sprintf((char *) &temp[5], "%s: %s", _prefs.node_name, &command[3]);
      temp[5 + MAX_TEXT_LEN] = 0;

      int len = strlen((char *) &temp[5]);
      auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, ch->channel, temp, 5 + len);
      if (pkt) {
        sendFlood(pkt);
        Serial.printf("   Sent to [%s]\n", getChannelName(_prefs.selected_channel_idx));
      } else {
        Serial.println("   ERROR: unable to send");
      }
    } else if (memcmp(command, "chsel ", 6) == 0) {  // select channel
      const char* ch_name = &command[6];
      int idx = findChannelByName(ch_name);
      if (idx >= 0) {
        _prefs.selected_channel_idx = idx;
        savePrefs();
        Serial.printf("   Channel '%s' selected\n", getChannelName(idx));
      } else {
        Serial.println("   ERROR: Channel not found");
      }
    } else if (memcmp(command, "list", 4) == 0) {  // show Contact list, by most recent
      int n = 0;
      if (command[4] == ' ') {  // optional param, last 'N'
        n = atoi(&command[5]);
      }
      scanRecentContacts(n, this);
    } else if (strcmp(command, "clock") == 0) {    // show current time
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      Serial.printf(   "%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      uint32_t secs = _atoi(&command[5]);
      setClock(secs);
    } else if (memcmp(command, "to ", 3) == 0) {  // set current recipient
      curr_recipient = searchContactsByPrefix(&command[3]);
      if (curr_recipient) {
        Serial.printf("   Recipient %s now selected.\n", curr_recipient->name);
      } else {
        Serial.println("   Error: Name prefix not found.");
      }
    } else if (strcmp(command, "to") == 0) {    // show current recipient
      if (curr_recipient) {
         Serial.printf("   Current: %s\n", curr_recipient->name);
      } else {
         Serial.println("   Err: no recipient selected");
      }
    } else if (strcmp(command, "advert") == 0) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        sendZeroHop(pkt);
        Serial.println("   (advert sent, zero hop).");
      } else {
        Serial.println("   ERR: unable to send");
      }
    } else if (strcmp(command, "reset path") == 0) {
      if (curr_recipient) {
        resetPathTo(*curr_recipient);
        saveContacts();
        Serial.println("   Done.");
      }
    } else if (memcmp(command, "card", 4) == 0) {
      Serial.printf("Hello %s\n", _prefs.node_name);
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        uint8_t len =  pkt->writeTo(tmp_buf);
        releasePacket(pkt);  // undo the obtainNewPacket()

        mesh::Utils::toHex(hex_buf, tmp_buf, len);
        Serial.println("Your MeshCore biz card:");
        Serial.print("meshcore://"); Serial.println(hex_buf);
        Serial.println();
      } else {
        Serial.println("  Error");
      }
    } else if (memcmp(command, "import ", 7) == 0) {
      importCard(&command[7]);
    } else if (memcmp(command, "set ch ", 7) == 0) {  // set ch <name> <hex_key> - MUST be before "set "
      const char* params = &command[7];
      char ch_name[32];
      char hex_key[65] = {0};
      
      // Check if it's a hashtag channel
      if (params[0] == '#') {
        if (sscanf(params, "%31s", ch_name) == 1) {
          // Hashtag channel - no key needed, just name
          if (setUserChannel(ch_name, "")) {
            savePrefs();
            Serial.printf("   Channel '%s' added (hashtag) - reboot to activate\n", ch_name);
          } else {
            Serial.println("   ERROR: Channel limit reached");
          }
        } else {
          Serial.println("   Usage: set ch #<name>  (for hashtag channel)");
        }
      } else if (sscanf(params, "%31s %64s", ch_name, hex_key) == 2) {
        // Regular channel with hex key (supports 128-bit or 256-bit keys)
        int key_len = strlen(hex_key);
        if (key_len == 32 || key_len == 64) {
          // Validate hex
          bool valid_hex = true;
          for (int i = 0; i < key_len; i++) {
            if (!mesh::Utils::isHexChar(hex_key[i])) {
              valid_hex = false;
              break;
            }
          }
          if (valid_hex) {
            if (setUserChannel(ch_name, hex_key)) {
              savePrefs();
              Serial.printf("   Channel '%s' added (%d-bit) - reboot to activate\n", ch_name, key_len * 4);
            } else {
              Serial.println("   ERROR: Channel limit reached");
            }
          } else {
            Serial.println("   ERROR: Invalid hex key");
          }
        } else {
          Serial.println("   ERROR: Key must be 32 (128-bit) or 64 (256-bit) hex characters");
        }
      } else {
        Serial.println("   Usage: set ch <name> <hex_key>  (32 or 64 hex chars)");
        Serial.println("          set ch #<name>           (hashtag channel)");
      }
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "af ", 3) == 0) {
        _prefs.airtime_factor = atof(&config[3]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "name ", 5) == 0) {
        StrHelper::strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name));
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lat ", 4) == 0) {
        _prefs.node_lat = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lon ", 4) == 0) {
        _prefs.node_lon = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "tx ", 3) == 0) {
        _prefs.tx_power_dbm = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "freq ", 5) == 0) {
        _prefs.freq = atof(&config[5]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "sf ", 3) == 0) {
        _prefs.sf = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "cr ", 3) == 0) {
        _prefs.cr = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "bw ", 3) == 0) {
        _prefs.bw = atof(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else {
        Serial.printf("  ERROR: unknown config: %s\n", config);
      }
    } else if (memcmp(command, "get", 3) == 0) {
      if (command[3] == 0 || command[3] == ' ') {  // "get" or "get <param>"
        const char* param = (command[3] == ' ') ? &command[4] : "";
        bool show_all = (param[0] == 0);
        
        if (show_all || strcmp(param, "name") == 0) {
          Serial.print("  name: "); Serial.println(_prefs.node_name);
        }
        if (show_all || strcmp(param, "lat") == 0) {
          Serial.print("  lat:  "); Serial.println(_prefs.node_lat, 6);
        }
        if (show_all || strcmp(param, "lon") == 0) {
          Serial.print("  lon:  "); Serial.println(_prefs.node_lon, 6);
        }
        if (show_all || strcmp(param, "freq") == 0) {
          Serial.print("  freq: "); Serial.print(_prefs.freq, 3); Serial.println(" MHz");
        }
        if (show_all || strcmp(param, "tx") == 0) {
          Serial.print("  tx:   "); Serial.print(_prefs.tx_power_dbm); Serial.println(" dBm");
        }
        if (show_all || strcmp(param, "sf") == 0) {
          Serial.print("  sf:   "); Serial.println(_prefs.sf);
        }
        if (show_all || strcmp(param, "cr") == 0) {
          Serial.print("  cr:   "); Serial.println(_prefs.cr);
        }
        if (show_all || strcmp(param, "bw") == 0) {
          Serial.print("  bw:   "); Serial.print(_prefs.bw, 1); Serial.println(" kHz");
        }
        if (show_all || strcmp(param, "af") == 0) {
          Serial.print("  af:   "); Serial.println(_prefs.airtime_factor, 2);
        }
        if (show_all || strcmp(param, "ch") == 0) {
          Serial.println("  Channels:");
          Serial.printf("    [0] Public%s%s\r\n", 
            _prefs.selected_channel_idx == 0 ? " *" : "",
            channel_muted[0] ? " (muted)" : "");
          for (int i = 0; i < MAX_GROUP_CHANNELS - 1; i++) {
            if (_prefs.channels[i].active) {
              Serial.printf("    [%d] %s%s%s\r\n", 
                i + 1, 
                _prefs.channels[i].name,
                _prefs.selected_channel_idx == (i + 1) ? " *" : "",
                channel_muted[i + 1] ? " (muted)" : "");
            }
          }
        }
      }
    } else if (memcmp(command, "del ch ", 7) == 0) {  // del ch <name>
      const char* ch_name = &command[7];
      if (strcasecmp(ch_name, "public") == 0) {
        Serial.println("   ERROR: Cannot delete Public channel");
      } else if (removeUserChannel(ch_name)) {
        // Reset to Public if we deleted selected channel
        const char* selected_name = getChannelNameFromPrefs(_prefs.selected_channel_idx);
        if (selected_name && strcasecmp(selected_name, ch_name) == 0) {
          _prefs.selected_channel_idx = 0;
        }
        savePrefs();
        Serial.printf("   Channel '%s' removed - reboot to apply\n", ch_name);
      } else {
        Serial.println("   ERROR: Channel not found");
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "mute ch ", 8) == 0) {  // mute ch <name>
      const char* ch_name = &command[8];
      int idx = findChannelByName(ch_name);
      if (idx >= 0) {
        channel_muted[idx] = true;
        if (idx > 0) {
          _prefs.channels[idx - 1].muted = true;
        }
        savePrefs();
        Serial.printf("   Channel '%s' muted\n", getChannelName(idx));
      } else {
        Serial.println("   ERROR: Channel not found");
      }
    } else if (memcmp(command, "unmute ch ", 10) == 0) {  // unmute ch <name>
      const char* ch_name = &command[10];
      int idx = findChannelByName(ch_name);
      if (idx >= 0) {
        channel_muted[idx] = false;
        if (idx > 0) {
          _prefs.channels[idx - 1].muted = false;
        }
        savePrefs();
        Serial.printf("   Channel '%s' unmuted\n", getChannelName(idx));
      } else {
        Serial.println("   ERROR: Channel not found");
      }
    } else if (memcmp(command, "mute", 4) == 0) {
      if (command[4] == 0 || command[4] == ' ') {  // "mute" or "mute <type>"
        const char* type = (command[4] == ' ') ? &command[5] : "advert";
        if (strcmp(type, "advert") == 0) {
          _prefs.mute_adverts = true;
          savePrefs();
          Serial.println("   ADVERT messages muted");
        } else {
          Serial.println("   ERROR: unknown mute type (try: advert, or 'ch <name>')");
        }
      }
    } else if (memcmp(command, "unmute", 6) == 0) {
      if (command[6] == 0 || command[6] == ' ') {  // "unmute" or "unmute <type>"
        const char* type = (command[6] == ' ') ? &command[7] : "advert";
        if (strcmp(type, "advert") == 0) {
          _prefs.mute_adverts = false;
          savePrefs();
          Serial.println("   ADVERT messages unmuted");
        } else {
          Serial.println("   ERROR: unknown unmute type (try: advert, or 'ch <name>')");
        }
      }
    } else if (memcmp(command, "reboot", 6) == 0) {
      Serial.println("Rebooting...");
      Serial.flush();  // ensure message is sent before reboot
      delay(100);
      board.reboot();
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   set {name|lat|lon|freq|tx|sf|cr|bw|af} {value}");
      Serial.println("   set ch <name> <hex_key>  - add channel (32 or 64 hex chars)");
      Serial.println("   set ch #<name>           - add hashtag channel");
      Serial.println("   get [{name|lat|lon|freq|tx|sf|cr|bw|af|ch}]");
      Serial.println("   del ch <name>            - delete channel");
      Serial.println("   card");
      Serial.println("   import {biz card}");
      Serial.println("   clock");
      Serial.println("   time <epoch-seconds>");
      Serial.println("   list {n}");
      Serial.println("   to <recipient name or prefix>");
      Serial.println("      (use TAB to autocomplete contact names)");
      Serial.println("   to");
      Serial.println("   send <text>");
      Serial.println("   chsel <name>             - select channel");
      Serial.println("   ch <text>                - send to selected channel");
      Serial.println("   mute ch <name>           - mute channel");
      Serial.println("   unmute ch <name>         - unmute channel");
      Serial.println("   mute [advert]");
      Serial.println("   unmute [advert]");
      Serial.println("   advert");
      Serial.println("   reset path");
      Serial.println("   reboot");
      Serial.println();
      Serial.println("Keyboard shortcuts:");
      Serial.println("   TAB - autocomplete contact name after 'to '");
      Serial.println("   ESC - clear current input line");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void loop() {
    BaseChatMesh::loop();

    int len = strlen(command);
    while (Serial.available() && len < sizeof(command)-1) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (len > 0) {  // have command to process
          command[len] = 0;  // null terminate
          Serial.println();  // echo newline
          handleCommand(command);
          Serial.print("\r> ");  // prompt for next command
          command[0] = 0;  // reset
          len = 0;
        }
      } else if (c == '\t' || c == 9) {  // Tab key for autocomplete
        command[len] = 0;  // ensure null termination
        handleTabCompletion(len);
      } else if (c == 27) {  // Escape key - clear current command
        // Clear the current line by overwriting with spaces
        Serial.print('\r');
        for (int i = 0; i < len + 2; i++) {  // +2 for "> "
          Serial.print(' ');
        }
        // Reset command buffer
        command[0] = 0;
        len = 0;
        // Show fresh prompt
        Serial.print("\r> ");
      } else if (c == 8 || c == 127) {  // backspace or delete
        if (len > 0) {
          len--;
          command[len] = 0;
          Serial.print("\b \b");  // backspace, space, backspace - erases character on terminal
        }
      } else {
        command[len++] = c;
        command[len] = 0;
        Serial.print(c);  // echo character as typed
      }
    }
    if (len >= sizeof(command)-1) {  // command buffer full
      Serial.println();
      Serial.println("   ERROR: command too long");
      command[0] = 0;
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);  // Give serial time to initialize

  board.begin();

  if (!radio_init()) { 
    halt(); 
  }

  fast_rng.begin(radio_get_rng_seed());

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

  radio_set_params(the_mesh.getFreqPref(), the_mesh.getBWPref(), the_mesh.getSFPref(), the_mesh.getCRPref());
  radio_set_tx_power(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvert(1200);
}

void loop() {
  the_mesh.loop();
}
