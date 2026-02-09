# CleanWalker — Strategic Direction

Last updated: 2026-02-09

## Current Phase: Pre-Revenue Digital Prototyping

### Core Principle
Zero physical spend until a deal is signed and money is in the bank. Everything digital first — simulation, CAD, ML training, renders.

### Priority Stack

**Tier 1 — Now:**
1. Website live (DONE — cleanwalkerrobotics.vercel.app)
2. Full cost model (DONE — see Research Reports below)
3. Perception model v1 (YOLO trained on TACO/TrashNet — provable AI demo)
4. Hardware architecture decisions (documented in ADRs)

**Tier 2 — This month:**
5. Robot CAD model + URDF for simulation
6. Simulation demo video (Gazebo/Isaac Sim)
7. Pitch deck + financial model
8. Austin TX municipal outreach (FY2027 planning window opens mid-2026)
9. Grant applications (DOT SMART, SBIR)

**Tier 3 — After signed deal:**
10. Physical prototype build (~3 iterations to pilot-ready)
11. Dashboard app (fleet management)
12. Fleet coordinator service
13. Production firmware

### Key Decisions Made
| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-02-09 | Dashboard app deprioritized | No customers = no fleet to manage |
| 2026-02-09 | Digital-first prototyping | No capital for physical builds yet |
| 2026-02-09 | AGPL-3.0 license | Open source with strong copyleft |
| 2026-02-09 | Austin TX as first pilot city | SF caps at 3 devices; TX preempts local PDD regulation; Austin already has Avride robots |
| 2026-02-09 | RaaS at $3,000-3,500/mo target pricing | Still 55-65% cheaper than human labor; well within gov budgets |
| 2026-02-09 | Tariff mitigation priority | Chinese actuators hit with ~49% stacked tariffs; engage customs broker |

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
| `pilot-financial-model.md` | Total investment: ~$363K. Pilot revenue @$1,200/mo: $72K. First pilot is a loss — that's normal |

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
6. Austin Resource Recovery Department as entry point
7. Target: Lady Bird Lake trail or downtown 6th Street corridor
