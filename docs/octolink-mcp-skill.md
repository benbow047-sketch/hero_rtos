# OctoLink MCP Skill

Guide for Claude to use OctoLink MCP tools for embedded target debugging via GDB/OpenOCD.

## When to Use

Use OctoLink MCP when you need to:

- Inspect target memory, variables, or registers during debug sessions
- Read sensor data, peripheral registers, or RTOS state from an embedded target
- Set/delete breakpoints and control execution (step, continue, reset)
- Monitor multiple variables in a single safe batch read
- Communicate with the target via serial port

**Do not use** when the target is not connected or GDB is not running. Always check `get_debug_context` or `get_gdb_status` first.

## Read-Only Flow (No Risk)

Reading variables/memory is safe and requires no confirmation.

### Single expression

```
Tool: gdb_evaluate_expression
Args: { "expression": "myVariable" }
```

### Batch read (up to 256 expressions)

```
Tool: gdb_evaluate_expressions
Args: { "expressions": ["var1", "var2", "sensor.temp"], "timeoutMs": 2000 }
```

### Safe batch read with auto halt/resume

When the target may be running, use `gdb_read_expressions_safe` to automatically halt before reading and optionally resume after:

```
Tool: gdb_read_expressions_safe
Args: {
  "expressions": ["uwTick", "sensorValue", "motorPWM"],
  "haltIfRunning": true,
  "resumeAfterRead": true,
  "allowReadWhileRunning": false
}
```

If `gdb_read_expressions_safe` returns `readAttempted: false`, do not retry single-expression reads and do not call `gdb_evaluate_expression` immediately. The target could not be halted cleanly, so direct reads are likely to timeout or produce a partial snapshot.

When the response includes `recommendedNextTool: "target_reset_halt"`, explain that a reset-halt will reset the embedded target and ask the user before proceeding. If the user agrees, call:

```
Tool: target_reset_halt
Args: { "confirm": true }
```

After reset-halt succeeds, call `gdb_evaluate_expressions` or `gdb_read_expressions_safe` again to collect a coherent snapshot.

### Read memory

```
Tool: read_memory_u32
Args: { "address": "0x40021000" }

Tool: read_memory_bytes
Args: { "address": "0x20000000", "length": 64 }
```

## Write Variable Flow (Destructive)

Writing to target memory **requires `confirm: true`**. Without it, the tool returns an error explaining the confirmation requirement.

### Standard write

```
Tool: write_gdb_variable
Args: {
  "expression": "motorSpeed",
  "value": "1500",
  "confirm": true
}
```

The tool writes the value, then reads it back to verify. The response includes `verified: true/false` and the `readBackValue`.

## Confirmation Convention

State-changing operations require `"confirm": true` in the arguments:

| Tool | Requires confirm |
|------|-----------------|
| `gdb_evaluate_expression` | No |
| `gdb_evaluate_expressions` | No |
| `gdb_read_expressions_safe` | No |
| `read_memory_u32` / `read_memory_bytes` | No |
| `gdb_list_breakpoints` | No |
| `write_gdb_variable` | **Yes** |
| `gdb_set_breakpoint` | **Yes** |
| `gdb_delete_breakpoint` | **Yes** |
| `target_continue` | **Yes** |
| `target_interrupt` | No (halting is safe) |
| `target_step` / `target_next` | **Yes** |
| `target_reset_halt` / `target_reset_run` | **Yes** |
| `serial_open` / `serial_close` / `serial_write` | **Yes** |

Always inform the user what will happen before calling a confirm-required tool.

## Safety Tips

1. **Check before you act.** Call `get_debug_context` or `get_gdb_status` to verify GDB is running and the target state before issuing commands.

2. **Prefer safe reads.** Use `gdb_read_expressions_safe` with `haltIfRunning: true` and `allowReadWhileRunning: false` when you are unsure if the target is halted. If it returns `readAttempted: false`, stop and ask the user before reset-halt.

3. **Do not write without reason.** Only use `write_gdb_variable` when explicitly asked to modify target state. Always explain the side effect to the user.

4. **Batch efficiently.** Use `gdb_evaluate_expressions` (up to 256) instead of looping single `gdb_evaluate_expression` calls. The server acquires the GDB lock once and auto-chunks at 32.

5. **Respect the lock.** All GDB tools acquire the command lock internally. Do not attempt to parallelize GDB commands from the client side; they will serialize.

6. **Serial port lifecycle.** Open with `serial_open` before writing. Close with `serial_close` when done. Check `get_serial_status` for current state.

7. **Breakpoint hygiene.** Use `temporary: true` for one-shot breakpoints. Clean up breakpoints with `gdb_delete_breakpoint` when no longer needed.

## Example: Inspect RTOS Task State

```
# 1. Check debug subsystem is ready
Tool: get_debug_context

# 2. Safe-read task control block fields
Tool: gdb_read_expressions_safe
Args: {
  "expressions": [
    "pxCurrentTCB->pcTaskName",
    "pxCurrentTCB->uxPriority",
    "pxCurrentTCB->pxTopOfStack",
    "uxTopReadyPriority"
  ],
  "haltIfRunning": true,
  "resumeAfterRead": true
}
```

## Example: Set Breakpoint and Continue to Hit

```
# 1. Set breakpoint
Tool: gdb_set_breakpoint
Args: { "location": "main.c:42", "confirm": true }

# 2. Continue execution
Tool: target_continue
Args: { "confirm": true }

# 3. (Later) Read local variables after breakpoint hit
Tool: gdb_evaluate_expressions
Args: { "expressions": ["localVar", "ptr->field"] }
```

## Example: Memory Dump

```
Tool: read_memory_bytes
Args: { "address": "0x20000000", "length": 128 }
```

Returns hex string, raw bytes array, and u32 little-endian interpretation of the first 4 bytes.
