# AI-assisted development

This project is built with LLM assistance (DeepSeek / Claude). This file records
**what** was delegated, **what** was verified by hand, and **where the AI was
wrong** — the last part is the one that matters in an interview.

## Division of labour

| Area | Delegated to AI | Kept by hand |
|---|---|---|
| ROS 2 boilerplate | node skeleton, publishers/subscribers, launch file | topic names, QoS choice, node split |
| Geometry | first draft of the projection formula | frame conventions, sign of every term, the 10 unit tests |
| Safety logic | state enum and transition table draft | target-loss timeout, jump filter, what happens on reacquisition |
| CI / packaging | workflow YAML, .gitignore, Dockerfile | which checks are gating, dependency pins |
| Docs | wording, structure | every number, every limitation |

## Prompt library

Reusable prompts, kept because a good prompt is a tool:

**1. Debugging an environment error**

> I am on Ubuntu 22.04 + ROS 2 Humble + PX4 v1.15 running `<command>`.
> Error: `<full log>`. Give me the most likely cause, one command to confirm it,
> and the fix. Do not give generic advice; if information is missing, list the
> three things you need from me first.

**2. Generating tests for a pure function**

> Here is `pixel_to_ground(u, v, h, K, yaw, image_up_is_forward)` and its frame
> conventions: `<paste docstring>`. Write pytest cases for: image centre, one
> pixel right of centre, one below centre, yaw = 90°, h = 0, malformed K.
> For each case, write the expected value and the arithmetic that produces it
> in a comment, so I can check the reasoning instead of trusting the number.

**3. Analysing results without overclaiming**

> `results/landing_error.csv` has columns trial, target_id, error_m,
> latency_p50_ms, success. Compute median/mean/std, flag outliers, and give me
> one paragraph for the README that states only what the data supports. Do not
> use "significantly" or "dramatically".

## AI failure log

Every time the model produced something plausible but wrong, it goes here. An
empty table means the AI was never actually checked.

| # | What the AI produced | Why it was wrong | How it was caught |
|---|---|---|---|
| 1 | `north = forward * cos(yaw) - right * sin(yaw)` | yaw rotation applied with the wrong sign for REP-103 | unit test `test_yaw_90_rotates_the_offset_into_the_world_frame` |
| 2 | `cv2.dnn.NMSBoxes` fed `[x1, y1, x2, y2]` | the function expects `[x, y, w, h]`; boxes silently collapsed | visual check of the detection overlay |
| 3 | _add yours_ | | |

## Rules I follow

1. **No AI-generated numbers.** Every value in the README comes from a CSV.
2. **No code I cannot explain.** If asked "why this line", I must answer without
   opening the file.
3. **No sensitive data in prompts.** No student ID, no internal IPs, no
   unpublished project names.
4. **AI output is a draft, tests are the review.** A function without a test is
   not verified, no matter who wrote it.
