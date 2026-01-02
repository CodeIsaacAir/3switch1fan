# Firmware Requirements: MCP Command Acknowledgment

## Overview
Add minimal support for command acknowledgment via 8-byte numeric correlation IDs to enable deterministic feedback for MCP (Model Context Protocol) clients.

## Memory Constraints
- **NO UUID strings** (16 bytes + overhead)
- **8-byte numeric ID only** (uint64_t)
- **NO flash persistence** - store in RAM only
- **Echo once, discard immediately**

---

## 1. MQTT Command Format Changes

### Current Format
```json
{
  "type": "gang_control",
  "dp_id": 123,
  "action": 1
}
```

### New Format (Backward Compatible)
```json
{
  "type": "gang_control",
  "dp_id": 123,
  "action": 1,
  "cid": 18446744073709551615,    // NEW: 8-byte numeric correlation ID (optional)
  "intent": "ai_initiated"         // NEW: optional string for analytics (optional)
}
```

**Requirements:**
- `cid` field is **optional** - if missing, device behaves as before (no ACK)
- `intent` field is **optional** - if missing, omit from status response
- Parse `cid` as **uint64_t** (8 bytes)
- Store `cid` in RAM only (static variable, not in NVS/flash)

---

## 2. MQTT Status Format Changes

### Current Format
```json
{
  "s": 5,
  "seq": 42
}
```

### New Format (Backward Compatible)
```json
{
  "s": 5,
  "seq": 42,
  "cid": 18446744073709551615,    // NEW: Echo correlation_id from command (if present)
  "intent": "ai_initiated"         // NEW: Echo intent from command (if present)
}
```

**Requirements:**
- Include `cid` **only if** it was present in the command
- Include `intent` **only if** it was present in the command
- Echo exact same values received in command
- **Discard `cid` immediately after publishing** - do not store

---

## 3. Implementation Details

### File: `main/network.c`

#### Changes in `mqtt_event_handler()` - MQTT_EVENT_DATA case

**Location:** Around line 202-249 (command topic handling)

**Current Code:**
```c
if(strcmp(mqtt_topic, command_topic) == 0){
    // Parse JSON
    json_t const *commandType = json_getProperty(root, "type");
    if (strcmp(json_getValue(commandType), "gang_control") == 0) {
        json_t const *dp_id = json_getProperty(root, "dp_id");
        json_t const *action = json_getProperty(root, "action");
        // ... forward to MCU
    }
}
```

**Required Changes:**
```c
// Add static variable to store correlation_id (RAM only, not persistent)
static uint64_t pending_correlation_id = 0;
static bool has_pending_cid = false;
static char pending_intent[32] = {0};  // Max 31 chars + null terminator
static bool has_pending_intent = false;

if(strcmp(mqtt_topic, command_topic) == 0){
    // Reset pending correlation data
    pending_correlation_id = 0;
    has_pending_cid = false;
    has_pending_intent = false;
    memset(pending_intent, 0, sizeof(pending_intent));
    
    // Parse JSON
    json_t const *commandType = json_getProperty(root, "type");
    if (strcmp(json_getValue(commandType), "gang_control") == 0) {
        json_t const *dp_id = json_getProperty(root, "dp_id");
        json_t const *action = json_getProperty(root, "action");
        
        // NEW: Parse optional correlation_id
        json_t const *cid = json_getProperty(root, "cid");
        if (cid) {
            // Parse as uint64_t (8-byte integer)
            pending_correlation_id = (uint64_t)json_getInteger(cid);
            has_pending_cid = true;
        }
        
        // NEW: Parse optional intent
        json_t const *intent = json_getProperty(root, "intent");
        if (intent) {
            const char *intent_value = json_getValue(intent);
            if (intent_value && strlen(intent_value) < sizeof(pending_intent)) {
                strncpy(pending_intent, intent_value, sizeof(pending_intent) - 1);
                pending_intent[sizeof(pending_intent) - 1] = '\0';
                has_pending_intent = true;
            }
        }
        
        // Forward to MCU (existing code)
        esp_err_t ret = publishStatusToMCU((uint8_t)json_getInteger(dp_id), (uint8_t)json_getInteger(action));
        if(ret == ESP_OK){
            xEventGroupSetBits(events_EventGroup, EVENT_DP_STATUS_MCU_COMMAND_BIT);
        }
    }
}
```

### File: `main/state_operational.c`

#### Changes in `operational_event()` - EVENT_DP_STATUS_MCU_COMMAND case

**Location:** Around line 261-274 (status publishing)

**Current Code:**
```c
case EVENT_DP_STATUS_MCU_COMMAND:
{
    static int seq = 0;
    static char dpStatus_message[MQTT_STATUS_BUFFER_SIZE];
    seq++;
    size_t dpStatus_size = snprintf(NULL, 0, "{\"s\": %d , \"seq\": %d}", 
                                     data->dpStatus.dpStatus, seq%1000);
    snprintf(dpStatus_message, (dpStatus_size + 1), 
             "{\"s\": %d , \"seq\": %d}", 
             data->dpStatus.dpStatus, seq%1000);
    if(data_lifetime->cloud_status) 
        esp_mqtt_client_publish(mqtt_client, status_topic, dpStatus_message, 
                                strlen(dpStatus_message), 1, 0);
}
break;
```

**Required Changes:**
```c
// Declare extern for pending correlation data (defined in network.c)
extern uint64_t pending_correlation_id;
extern bool has_pending_cid;
extern char pending_intent[32];
extern bool has_pending_intent;

case EVENT_DP_STATUS_MCU_COMMAND:
{
    static int seq = 0;
    static char dpStatus_message[MQTT_STATUS_BUFFER_SIZE];
    seq++;
    
    // Build status message with optional cid and intent
    if (has_pending_cid && has_pending_intent) {
        // Include both cid and intent
        size_t dpStatus_size = snprintf(NULL, 0, 
            "{\"s\": %d , \"seq\": %d, \"cid\": %llu, \"intent\": \"%s\"}", 
            data->dpStatus.dpStatus, seq%1000, 
            (unsigned long long)pending_correlation_id, pending_intent);
        snprintf(dpStatus_message, (dpStatus_size + 1), 
            "{\"s\": %d , \"seq\": %d, \"cid\": %llu, \"intent\": \"%s\"}", 
            data->dpStatus.dpStatus, seq%1000, 
            (unsigned long long)pending_correlation_id, pending_intent);
    }
    else if (has_pending_cid) {
        // Include only cid
        size_t dpStatus_size = snprintf(NULL, 0, 
            "{\"s\": %d , \"seq\": %d, \"cid\": %llu}", 
            data->dpStatus.dpStatus, seq%1000, 
            (unsigned long long)pending_correlation_id);
        snprintf(dpStatus_message, (dpStatus_size + 1), 
            "{\"s\": %d , \"seq\": %d, \"cid\": %llu}", 
            data->dpStatus.dpStatus, seq%1000, 
            (unsigned long long)pending_correlation_id);
    }
    else {
        // No cid - use original format (backward compatible)
        size_t dpStatus_size = snprintf(NULL, 0, 
            "{\"s\": %d , \"seq\": %d}", 
            data->dpStatus.dpStatus, seq%1000);
        snprintf(dpStatus_message, (dpStatus_size + 1), 
            "{\"s\": %d , \"seq\": %d}", 
            data->dpStatus.dpStatus, seq%1000);
    }
    
    // Publish status
    if(data_lifetime->cloud_status) {
        esp_mqtt_client_publish(mqtt_client, status_topic, dpStatus_message, 
                                strlen(dpStatus_message), 1, 0);
    }
    
    // CRITICAL: Discard correlation_id immediately after publishing
    // (Reset handled in network.c on next command, but clear here for safety)
    has_pending_cid = false;
    has_pending_intent = false;
    pending_correlation_id = 0;
    memset(pending_intent, 0, sizeof(pending_intent));
}
break;
```

---

## 4. Header File Changes

### File: `main/network.h`

Add extern declarations:
```c
// Correlation ID support (RAM only, not persistent)
extern uint64_t pending_correlation_id;
extern bool has_pending_cid;
extern char pending_intent[32];
extern bool has_pending_intent;
```

---

## 5. Memory Impact

**Additional RAM Usage:**
- `uint64_t pending_correlation_id`: 8 bytes
- `bool has_pending_cid`: 1 byte
- `char pending_intent[32]`: 32 bytes
- `bool has_pending_intent`: 1 byte
- **Total: ~42 bytes** (negligible)

**Flash Usage:** 0 bytes (no persistence)

---

## 6. Backward Compatibility

- **Commands without `cid`**: Device behaves exactly as before
- **Status without `cid`**: Backend handles both formats
- **No breaking changes**: Existing commands continue to work

---

## 7. Testing Checklist

- [ ] Command with `cid` → Status includes `cid`
- [ ] Command without `cid` → Status excludes `cid` (backward compatible)
- [ ] Command with `intent` → Status includes `intent`
- [ ] Command without `intent` → Status excludes `intent`
- [ ] Multiple commands → Each `cid` echoed correctly
- [ ] `cid` discarded after status publish (not persisted)
- [ ] Large `cid` values (max uint64_t) handled correctly
- [ ] `intent` string truncated if > 31 chars

---

## 8. Example Scenarios

### Scenario 1: Command with correlation_id
**Command:**
```json
{
  "type": "gang_control",
  "dp_id": 101,
  "action": 1,
  "cid": 12345678901234567890,
  "intent": "ai_initiated"
}
```

**Expected Status:**
```json
{
  "s": 5,
  "seq": 42,
  "cid": 12345678901234567890,
  "intent": "ai_initiated"
}
```

### Scenario 2: Command without correlation_id (backward compatible)
**Command:**
```json
{
  "type": "gang_control",
  "dp_id": 101,
  "action": 1
}
```

**Expected Status:**
```json
{
  "s": 5,
  "seq": 42
}
```

---

## 9. Notes for Firmware Team

1. **Minimal Changes**: Only 2 files need modification (`network.c`, `state_operational.c`)
2. **No Flash Writes**: All correlation data is RAM-only
3. **No State Machine Changes**: Existing flow unchanged
4. **Optional Fields**: Both `cid` and `intent` are optional - device works without them
5. **Memory Efficient**: Only 42 bytes additional RAM
6. **Simple Logic**: Parse → Store → Echo → Discard

---

## Questions?

Contact backend team for clarification on:
- JSON parsing library limitations
- Buffer size constraints
- Edge cases

