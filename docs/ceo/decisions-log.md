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

## ADR-007: Global Market Entry — EU + UAE First, Not US-Only
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Global market research revealed the US is NOT the optimal first market. Singapore, UAE, and Northern Europe offer better regulatory frameworks, more innovation funding, and fewer barriers. Veolia (56 countries, EUR 45B revenue) already trialed quadruped litter robots at Bondi Beach, Sydney. BEEAH (UAE) has deployed autonomous cleaners since 2019. No commercial quadrupedal litter-collecting robot exists anywhere in the world.
**Decision:** Phase 1: EU (Netherlands/Germany) + UAE. Phase 2: UK + Australia + Singapore. Phase 3: US + Japan + Korea. Austin TX remains the US target but moves to Phase 3.
**Consequence:** Broader opportunity set. EU Machinery Regulation 2023/1230 (Jan 2027) provides clear regulatory pathway. IFAT Munich (May 2026) becomes global launch event. Veolia partnership could open 56 countries with a single deal.

## ADR-008: Veolia as #1 Strategic Partner Target
**Date:** 2026-02-09
**Status:** Accepted
**Context:** Veolia (EUR 45B revenue, 56 countries, 215K employees) trialed "Scoop Doggy Dog" quadrupedal litter-collecting robots at Bondi Beach, Sydney. They are demonstrably interested in this exact technology. A single Veolia partnership could deploy CleanWalker across dozens of cities globally.
**Decision:** Veolia ANZ (Australia team that ran Bondi trial) is the #1 outreach priority. Approach them before any other partner. BEEAH Group (UAE) is #2.
**Consequence:** Changes outreach sequence. Veolia approach requires: simulation demo video, live AI demo, and professional pitch deck ready BEFORE contact.

## ADR-009: IFAT Munich 2026 as Global Launch Event
**Date:** 2026-02-09
**Status:** Pending Action
**Context:** IFAT Munich (May 4-7, 2026) is the world's largest environmental technology trade show — 3,000+ exhibitors, 140,000+ attendees from 60+ countries. Every major waste company, municipal buyer, and equipment manufacturer attends. Has a startup exhibit area.
**Decision:** Register for IFAT 2026 startup exhibit area immediately. This is CleanWalker's global launch event. Budget EUR 5,000-15,000 for small booth.
**Consequence:** 3 months away — requires immediate action. Need simulation demo, AI demo, and marketing materials ready by April 2026. Contact application@ifat.de for startup area.
