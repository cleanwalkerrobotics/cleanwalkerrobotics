# CleanWalker — Strategic Direction

Last updated: 2026-02-09

## Current Phase: Pre-Revenue Digital Prototyping

### Core Principle
Zero physical spend until a deal is signed and money is in the bank. Everything digital first — simulation, CAD, ML training, renders.

### Priority Stack

**Tier 1 — Now:**
1. Website live (DONE — cleanwalkerrobotics.vercel.app)
2. Full cost model (DONE — see Research Reports below)
3. Global market research (DONE — 5 regional reports complete)
4. Perception model v1 (YOLO trained on TACO/TrashNet — provable AI demo)
5. Hardware architecture decisions (documented in ADRs)

**Tier 2 — This month:**
6. Robot CAD model + URDF for simulation
7. Simulation demo video (Gazebo/Isaac Sim)
8. Pitch deck + financial model (incorporate global market data)
9. ~~IFAT Munich 2026~~ — SCRATCHED per Maurits (2026-02-09). Not pursuing.
10. Veolia ANZ outreach (Bondi Beach quadruped trial team)
11. BEEAH Group outreach (MENA gateway)
12. Grant applications (EIC Accelerator "Physical AI", DOT SMART, SBIR)
13. Austin TX municipal outreach (FY2027 planning window opens mid-2026)

**Tier 3 — After signed deal:**
14. Physical prototype build (~3 iterations to pilot-ready)
15. Dashboard app (fleet management)
16. Fleet coordinator service
17. Production firmware
18. EU Machinery Regulation 2023/1230 compliance (effective Jan 2027)

### Key Decisions Made
| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-02-09 | Dashboard app deprioritized | No customers = no fleet to manage |
| 2026-02-09 | Digital-first prototyping | No capital for physical builds yet |
| 2026-02-09 | AGPL-3.0 license | Open source with strong copyleft |
| 2026-02-09 | Austin TX as US pilot city | SF caps at 3 devices; TX preempts local PDD regulation; Austin already has Avride robots |
| 2026-02-09 | RaaS at $3,000-3,500/mo target pricing | Still 55-65% cheaper than human labor; well within gov budgets |
| 2026-02-09 | Tariff mitigation priority | Chinese actuators hit with ~49% stacked tariffs; engage customs broker |
| 2026-02-09 | Global market entry strategy | Phase 1: EU + UAE. Phase 2: UK + Australia + Singapore. Phase 3: US + Japan + Korea |
| 2026-02-09 | Veolia = #1 strategic partner | Already trialed quadruped litter robots at Bondi Beach; 56 countries; EUR 45B revenue |
| 2026-02-09 | ~~IFAT Munich 2026~~ — SCRATCHED | Per Maurits: not pursuing. Removed from priority stack. |

### Global Market Intelligence

**TAM/SAM/SOM:**
| Metric | 2025 | 2030 |
|--------|------|------|
| TAM (autonomous outdoor litter robots) | $3-5B | $8-12B |
| SAM (developed markets, quadrupedal niche) | $1.0-1.7B | $2.6-4.3B |
| SOM (5-year cumulative) | — | $160-400M |

**Critical Finding:** No commercial quadrupedal litter-collecting robot exists. VERO (IIT Genoa) is academic. Veolia's "Scoop Doggy Dog" at Bondi Beach was a proof-of-concept. CleanWalker = first-to-market.

**Top 10 Global Cities by Opportunity:**
| Rank | City | Key Reason |
|------|------|------------|
| 1 | Singapore | NEA grants, zero tariffs, PSG 80% co-funding, best regulatory framework |
| 2 | Dubai/Abu Dhabi | Already deploying autonomous sweepers, BEEAH, USD-pegged |
| 3 | Amsterdam | "Clean & Waste Free Framework 2025-2028", smart city platform |
| 4 | Helsinki | Testbed Helsinki, most permissive EU regulations |
| 5 | Seoul | $158M "Robot City" investment, zero tariffs (KORUS FTA) |
| 6 | London | GBP 500M+ cleaning budget, ISWA 2026 host |
| 7 | Munich | IFAT 2026 host city, gateway to DACH market |
| 8 | Sydney | Veolia's Bondi Beach trial, AUD 86M budget, zero tariffs |
| 9 | Copenhagen | "Green capital" brand, Odense robotics cluster |
| 10 | Tokyo | Largest APAC market, Level 4 sidewalk robots legal since 2023 |

**Global Market Entry Sequence:**
- **Phase 1 (2026-2027):** Netherlands + UAE + Germany (IFAT launch)
- **Phase 2 (2028-2029):** UK + Australia + Singapore + wider Europe
- **Phase 3 (2029-2031):** US + Japan + South Korea + Saudi Arabia

**Top 3 Strategic Partnerships:**
| Partner | Type | Why |
|---------|------|-----|
| Veolia (EUR 45B, 56 countries) | Distribution + Customer | Already trialed quadruped litter robots — they WANT this |
| BEEAH Group (UAE) | Innovation Partner | Smart city DNA, MENA gateway, deploying autonomous cleaners since 2019 |
| WM/Republic Services (US) | Customer | $443M tech investment budget, largest US market |

**Key Trade Shows 2026:**
| Event | Date | Location | Priority |
|-------|------|----------|----------|
| ~~IFAT Munich~~ | ~~May 4-7~~ | ~~Munich~~ | SCRATCHED — not pursuing |
| ICRA 2026 | Jun 1-5 | Vienna | HIGH — technical credibility |
| Smart City Expo | Nov 3-5 | Barcelona | HIGH — 850+ city delegations |
| ISWA Congress | Nov 9-12 | London | HIGH — waste industry executives |

### Research Reports (all in docs/)
| Report | Key Finding |
|--------|------------|
| `hardware-bom-research.md` | Prototype: $10.2K, production @1000: $5.5K. Actuators = 50-60% of BOM |
| `ml-compute-costs.md` | Training: $0-80 (free GPUs), inference: $249 (Jetson Orin Nano) |
| `market-research.md` | No quadrupedal competitors. SF spends $47.8M/yr on cleaning. RaaS model preferred |
| `product-spec-research.md` | 50kg, IP65, dog-like design. ODRI Solo-12 best reference. Safety certs: $195-380K |
| `assembly-iteration-costs.md` | 60hrs/unit assembly. 3-5 iterations to pilot-ready. Recommended path: $287K |
| `logistics-transport-costs.md` | $64K for 10-unit pilot logistics. Tariffs = $24.7K (!). Per-robot logistics: $6.4K |
| `certification-pilot-costs.md` | Austin min cert: $18-43K. FCC only needs SDoC ($2.8-7K). ISO/UL not needed for pilot |
| `pilot-financial-model.md` | Total investment: ~$363K. Pilot revenue @$3,500/mo: $210K. Pilot P&L: -$153K |
| `market-research-europe.md` | Helsinki #1 EU city. EUR 15-20B EU cleaning market. EU Machinery Reg 2023/1230 effective Jan 2027 |
| `market-research-asia-pacific.md` | Singapore #1 APAC. Zero tariffs. NEA grants. No quadruped competitors anywhere in APAC |
| `market-research-mena-africa.md` | Abu Dhabi already deploying autonomous sweepers. BEEAH = key partner. $90B+ combined market |
| `market-research-americas.md` | Toronto CAD $50M litter ops. Calgary $7M spring cleanup. Chile zero tariffs. Caribbean resort opportunity |
| `market-research-global-synthesis.md` | Global TAM $3-5B. Veolia = #1 partner. No commercial quadruped litter robot exists. IFAT = launch event |

### Revised Pilot Economics (at $3,500/mo pricing)
| Metric | Value |
|--------|-------|
| Total investment to pilot | ~$363K |
| Pilot revenue (10 units, 6mo, $3,500/mo) | $210K |
| Pilot P&L | -$153K |
| Cash needed before first payment | ~$275K |
| Value displaced (vs human labor) | $500K-875K/yr |
| City savings vs status quo | 55-65% |

### What We Need to Close First Deal
1. Professional website with robot renders (not stock photos)
2. Simulation video showing the robot detecting + picking up trash
3. Live AI demo (trash detection on real video footage)
4. Detailed cost proposal with per-unit and fleet pricing
5. Pilot proposal: 10 units, 1 zone, 6 months, clear KPIs
6. **Primary targets:** Veolia ANZ (Bondi Beach team), BEEAH (Sharjah), Amsterdam Smart City
7. **US target:** Austin Resource Recovery Department — Lady Bird Lake trail or 6th Street corridor
8. **EU credibility:** ~~IFAT Munich~~ — SCRATCHED. Alternative: ICRA Vienna (Jun 2026) or Smart City Expo Barcelona (Nov 2026)
