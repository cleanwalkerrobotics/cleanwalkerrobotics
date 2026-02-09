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

## ADR-004: Austin TX as First Pilot City
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Research showed SF caps autonomous devices at 3 per company (Section 794). Texas preempts local PDD regulation (SB 969). Austin already hosts Avride autonomous delivery robots. Austin FY starts Oct 1 — planning window for FY2027 opens mid-2026.
**Decision:** Target Austin, TX for first pilot. Entry point: Austin Resource Recovery Department. Target zones: Lady Bird Lake trail or downtown 6th Street corridor.
**Consequence:** Favorable regulatory environment. No device count caps. 110 lb weight limit (we're ~66 lbs). $100K minimum insurance requirement.

## ADR-005: RaaS Pricing at $3,000-3,500/month
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Original model used $1,200/mo (40% pilot discount). Government budgets can absorb much higher pricing. SF spends $47.8M/yr on street cleaning. Each robot displaces 1.5-2.5 FTEs ($55-85K/yr each). Even at $3,500/mo, cities save 55-65% vs human labor.
**Decision:** Target $3,000-3,500/month per robot for pilot pricing. Offer 10-15% introductory discount, not 40%.
**Consequence:** 10-unit 6-month pilot generates $180-210K (vs $72K at old pricing). Pilot P&L improves from -$291K to -$153K. Still well under city procurement thresholds.

## ADR-006: Tariff Mitigation — Engage Customs Broker
**Date:** 2026-02-09
**Status:** Pending Action
**Context:** Chinese actuators (CubeMars) face ~49% stacked tariffs (MFN + Section 301 + Reciprocal + IEEPA). That's $2,352/robot in duties alone, $24.7K for 10 units. This was a hidden cost not in the original BOM.
**Decision:** Engage a licensed customs broker immediately ($500-1,500) to confirm HTS classifications, check Section 301 exclusions, and evaluate domestic PCB fabrication. Long-term: source actuators from non-China suppliers.
**Consequence:** Could save $12K+ on actuator tariffs. Domestic PCBs eliminate ~45% tariff on JLCPCB boards.
