# CleanWalker — Architecture Decision Records (Business)

## ADR-001: Digital-First Prototyping Strategy
**Date:** 2026-02-09
**Status:** Accepted
**Context:** No outside capital. Need to minimize spend while maximizing credibility.
**Decision:** All prototyping happens digitally (CAD, simulation, ML training) until a deal is signed with payment guaranteed.
**Consequence:** Longer time to physical demo, but zero financial risk. Sales pitch relies on simulation + AI demos.

## ADR-002: Dashboard App Deprioritized
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Dashboard (fleet management) was scaffolded but has no users.
**Decision:** No further development on dashboard until first customer is signed.
**Consequence:** Code stays in repo, ready to build out when needed. Engineering time redirected to cost research and digital prototyping.

## ADR-003: AGPL-3.0 License
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Want open source for credibility and community, but need protection against competitors copying.
**Decision:** AGPL-3.0 — strong copyleft, requires derivative works to also be open source.
**Consequence:** Competitors can't use our code in proprietary products. May deter some enterprise contributors, but aligns with values.
