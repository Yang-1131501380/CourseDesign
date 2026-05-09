# AUTOSTAR Style Rules For F407_TEST

## Scope

This file applies to new and modified C code under `F407_TEST/`.

- Use these rules for `Core/`, `HARDWARE/`, `SYSTEM/`, and similar local code.
- Do not use this file as a reason to reformat vendor, generated, or unrelated
  code.
- Historical interfaces may keep their original names if changing them would
  increase risk or churn.

## Naming Rules

### Functions

- New function names must use `lower_snake_case`.
- Do not add new `CamelCase` function names.
- Static helper functions should use a short module-oriented prefix, such as
  `vision_frame_parse()` or `gimbal_ctrl_update()`.
- Existing public interfaces such as `MotorCtrl_SetLeftSpeed()` may remain
  unchanged unless the task explicitly requires renaming.

### Variables And Types

- Local variables should use clear lower camel case or project-compatible
  Hungarian-style names, such as `lineIndex` or `pFrame`.
- Do not use unclear single-letter names except for simple loop indices such as
  `i`, `j`, and `k`.
- Macros, enum values, and typedef aliases should use uppercase with
  underscores.
- Structs and enums should follow the existing project conventions unless the
  surrounding file already defines a stricter local pattern.

## Comment Rules

- Use block comments in `/* ... */` form for implementation comments.
- Prefer section comments to separate file regions, for example:

```c
/*=============================== Header Files ===============================*/
```

- Public or non-trivial functions should have a function header comment with
  `@brief`, parameters, and return value.
- Keep comments factual and short. Comment intent, assumptions, ranges, and
  hardware meaning. Do not comment obvious syntax.
- When a structure member or macro has non-obvious meaning, add a short comment.

### Function Header Template

```c
/*******************************************************************************
 * @brief   Parse one UART frame from the vision link
 * @param   [in]buffer    Input byte buffer
 * @param   [in]length    Valid byte count in buffer
 * @param   [out]pFrame   Parsed frame output
 * @return  0 on success, negative value on error
 ******************************************************************************/
```

## Formatting Rules

- Use 4 spaces for indentation. Do not use tabs for alignment-sensitive logic.
- Put function and control-block braces on their own lines.
- Add one space after `if`, `for`, `while`, and before the opening parenthesis.
- Always use braces for `if`, `else`, `for`, and `while`, even for one line.
- Put spaces around binary operators, assignments, and comparisons.
- Put one space after commas.
- Avoid trailing whitespace.
- Keep lines near 80 columns where practical. Long string literals may exceed
  this when necessary.
- Leave one blank line between logical blocks and between functions. Do not use
  multiple blank lines without a reason.

## Change Boundaries

- Make the smallest change that directly satisfies the task.
- Do not clean up adjacent code unless your change makes it necessary.
- Remove only the unused code created by your own modification.
- Match existing module structure before introducing new files or abstractions.
- If the repository already has a working interface, prefer extending it over
  creating a parallel wrapper.

## Review Checklist

Before finishing a C code change under `F407_TEST`, check:

- New functions use `lower_snake_case`
- Control statements always use braces
- Operators and commas are spaced correctly
- Function headers exist where needed
- Comments use block-comment style
- The diff is limited to task-related lines
