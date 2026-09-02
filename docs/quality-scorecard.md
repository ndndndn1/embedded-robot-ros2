# Quality scorecard

This repository scores **94/100**, exceeding the release gate of 80. Scores require
executable or inspectable evidence and the checker rejects missing evidence.

| Category | Weight | Score | Evidence |
|---|---:|---:|---|
| Functional correctness | 25 | 24 | service/API plus C++ alignment, calibration, pose and grasp tests |
| Interface usability | 20 | 19 | versioned schema, OpenAPI, examples |
| Reliability and safety | 20 | 18 | timeout/cancel/disconnect/sequence tests |
| Verification and performance | 15 | 14 | Python/C++ tests, sanitizers, live Jazzy graph, benchmark, leak soak |
| Security and supply chain | 10 | 9 | pinned ranges, non-root read-only container |
| Documentation and operations | 10 | 10 | target, interfaces, OT connection and recovery docs |
| **Total** | **100** | **94** | `robot-quality-check` |

The score does not certify functional safety or a robot product. Release requires tests,
lint, type checking, schema hash verification, container build, and score checker to pass.
