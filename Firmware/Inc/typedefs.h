#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef float f32;
typedef double f64;


// const: function is completely pure, with no side effects or references to mutable program state.
#define __fn_const __attribute__((const))

// pure: function is mostly pure, with no side effects, but may reference mutable nonvolatile program state
#define __fn_pure __attribute__((pure))

// flatten: all calls inside this function are inlined if possible. Useful for speed and register pressure.
#define __fn_flatten __attribute__((flatten))

// hot: function is called often - compiler will aggressively optimize and prioritize function locality
#define __fn_hot __attribute__((hot))

// leaf: function will never access any function calls. Good for dataflow analysis and reducing register pressure.
#define __fn_leaf __attribute__((leaf))

// noreturn: function loops forever / stops microcontroller
#define __fn_noreturn __attribute__((noreturn))

// used: force emission of code even if not referenced
#define __fn_used __attribute__((used))

// inline: force placement of function inline (as opposed to suggesting inline placement, which is what `inline` does)
#define __fn_inline inline __attribute__((always_inline))

// maybe_unused: a variable that may be deliberately unused (debug hooks, toggled-off scaffolding).
// Silences -Wunused-variable / -Wunused-but-set-variable without deleting the read. Guarded since
// some libc <sys/cdefs.h> also define __maybe_unused.
#ifndef __maybe_unused
#define __maybe_unused __attribute__((unused))
#endif


#define __ccmram __attribute__((section (".ccmram")))