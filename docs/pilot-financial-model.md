# CleanWalker Robotics -- Pilot Financial Model

> **⚠️ UNDER REVIEW:** Unit costs based on consumer-grade components. Industrial-grade recalculation in progress.

**Date:** February 2026
**Version:** 1.0
**Purpose:** Answer the CEO's question: "If I charge a customer X for a pilot, what does it actually cost me, and what do I need upfront?"

---

## Table of Contents

1. [Phase 1: Pre-Pilot Development (Month 1-6)](#phase-1-pre-pilot-development-month-1-6)
2. [Phase 2: Pilot Production (Month 6-8)](#phase-2-pilot-production-month-6-8)
3. [Phase 3: Pilot Deployment (Month 8-10)](#phase-3-pilot-deployment-month-8-10)
4. [Phase 4: Pilot Operations (Month 10-16)](#phase-4-pilot-operations-month-10-16)
5. [Revenue Model](#revenue-model)
6. [Summary: The CEO's One-Pager](#summary-the-ceos-one-pager)
7. [Cash Flow Timeline](#cash-flow-timeline)
8. [Sensitivity Analysis](#sensitivity-analysis)
9. [Key Assumptions and Honest Disclaimers](#key-assumptions-and-honest-disclaimers)

---

## Phase 1: Pre-Pilot Development (Month 1-6)

This phase takes CleanWalker from current state (software only, zero hardware) to a pilot-ready robot design with 10 units built and tested.

### 1.1 Prototype Iterations

Not every iteration is a full rebuild. The first prototype is a full build. Subsequent iterations target specific subsystems that failed or underperformed during testing.

#### Iteration 1: Proof-of-Concept (Month 1-2)

Full build from scratch. Goal: Can it walk, see, and pick up a plastic bottle?

| Item | Cost | Notes |
|------|------|-------|
| Full BOM (1 unit at prototype pricing) | $10,224 | Per BOM research: actuators $5,188, gripper $705, perception $498, compute $249, power $450, bin $200, comms $80, PCB $120, frame $1,250, wiring $150, plus 15% contingency |
| Assembly labor (80 hrs @ $50/hr) | $4,000 | First unit takes much longer -- learning curve, debugging, wiring from scratch |
| 3D printing for enclosure and test parts | $300 | ASA/PETG for enclosure, PLA for test fixtures |
| Miscellaneous tools and supplies | $500 | Soldering equipment, CAN bus analyzer, bench power supply, hand tools |
| **Iteration 1 total** | **$15,024** | |

#### Iteration 2: Redesign Based on Testing (Month 2-4)

Partial rebuild. Expected failures from iteration 1: leg joint alignment, gripper reliability, wiring harness, thermal management. Does NOT require a full new unit -- reuses compute, battery, sensors, most of the frame.

| Item | Cost | Notes |
|------|------|-------|
| Replacement actuator (1-2 units, damaged in testing) | $800 | 1x AK70-10 ($499) + 1x AK60-6 ($299) as spares |
| Redesigned gripper (new Dynamixel + revised fingers) | $350 | New silicone mold, revised linkage, replacement servo |
| Revised PCB / wiring harness | $250 | New power distribution board iteration from JLCPCB |
| Revised CNC frame parts (2-3 brackets) | $300 | Xometry rush order for redesigned motor mounts |
| Assembly labor (40 hrs @ $50/hr) | $2,000 | Disassembly, modification, reassembly, testing |
| **Iteration 2 total** | **$3,700** | |

#### Iteration 3: Pre-Pilot Version (Month 4-6)

Final refinements. Goal: weatherproofing, reliability testing, extended runtime validation. This version should be close to what ships to the customer.

| Item | Cost | Notes |
|------|------|-------|
| Revised enclosure (improved IP65 sealing) | $400 | New 3D printed enclosure with improved gasket design, conformal coating on PCBs |
| Foot pad redesign | $150 | Custom rubber compound, revised mounting |
| Final wiring harness (production-style) | $200 | Clean harness with proper connectors, strain relief |
| Extended environmental testing supplies | $300 | Rain testing setup, temperature cycling (use lab oven) |
| Assembly labor (30 hrs @ $50/hr) | $1,500 | Final integration, sealing, testing |
| **Iteration 3 total** | **$2,550** | |

#### Total prototype iteration cost: $21,274

### 1.2 Engineering Costs (Month 1-6)

Two scenarios: Scrappy Founder (founder + 1 hire) vs. Professional Team.

#### Scenario A: Scrappy Founder

Founder does mechanical + electrical + firmware. One ML/software hire. No office -- uses garage/makerspace.

| Role | Monthly Cost | Months | Total | Notes |
|------|-------------|--------|-------|-------|
| Founder salary (taken or deferred) | $0 | 6 | $0 | Founder takes no salary, lives on savings or investment. This is the norm for pre-seed startups. |
| ML/Software engineer (1 FTE, contract) | $8,000/mo | 6 | $48,000 | Bay Area freelance rate for junior-mid ML engineer. Could be lower with equity compensation or offshore hire ($4-5K/mo). |
| Mechanical/electrical consultant (part-time) | $3,000/mo | 3 | $9,000 | 10-15 hrs/month for PCB review, mechanical design review. Not needed full 6 months. |
| **Total engineering (Scrappy)** | | | **$57,000** | |

#### Scenario B: Professional Team

Small full-time team in a modest office/lab.

| Role | Monthly Cost | Months | Total | Notes |
|------|-------------|--------|-------|-------|
| Founder/CEO (reduced salary) | $8,000/mo | 6 | $48,000 | Below-market but livable |
| Mechanical engineer (1 FTE) | $10,000/mo | 6 | $60,000 | Bay Area junior-mid rate, fully loaded |
| Electrical/firmware engineer (1 FTE) | $10,000/mo | 6 | $60,000 | Same |
| ML/Software engineer (1 FTE) | $10,000/mo | 6 | $60,000 | Same |
| **Total engineering (Professional)** | | | **$228,000** | |

### 1.3 Workspace and Tooling Setup

| Item | Scrappy | Professional | Notes |
|------|---------|-------------|-------|
| Workspace (6 months) | $3,000 | $18,000 | Scrappy: makerspace membership ($500/mo). Pro: small industrial unit ($3,000/mo). |
| Workbench, shelving, storage | $500 | $2,000 | |
| Soldering station + rework station | $400 | $1,200 | Already included in misc tools for prototype, but formalizing here |
| CAN bus debugger, oscilloscope, multimeter | $800 | $2,000 | Scrappy uses cheaper equipment |
| 3D printer (if not already owned) | $500 | $1,500 | Bambu Lab P1S (~$500) vs. industrial FDM |
| Safety equipment (PPE, fire extinguisher) | $200 | $500 | |
| **Total workspace/tooling** | **$5,400** | **$25,200** | |

### 1.4 ML Model Training

| Item | Cost | Notes |
|------|------|-------|
| Dataset (public, free: TACO + Litter + Drinking Waste) | $0 | ~25,000 images combined |
| Custom data collection (5,000 images, team effort) | $0 | Phone camera collection at local parks |
| Annotation (auto-label + review, 20,000 images) | $400 | Roboflow auto-label + manual review |
| Training compute (10-15 runs) | $80 | Kaggle free tier + a few Vast.ai runs at $4-8 each |
| Cloud storage (Cloudflare R2, 50 GB) | $5 | 6 months at ~$0.75/mo |
| **Total ML** | **$485** | ML is cheap. This is one of our advantages. |

### 1.5 Minimum Certifications for Pilot

For a US-based pilot with a cooperative municipality, we do NOT need full certification. We need enough to satisfy the city's risk management office and our insurance provider.

| Item | Cost | Notes |
|------|------|-------|
| FCC Part 15 (required -- robot has 4G + WiFi + BLE radios) | $8,000 | Using pre-certified modules (SIM7600G-H, Jetson's WiFi) reduces cost. Estimated $5K-$15K, using midpoint. |
| Basic electrical safety testing (UL pre-compliance) | $5,000 | Not full UL 3300 (that's $50K-$100K and takes 6+ months). Just enough for insurance sign-off. |
| IP65 ingress protection testing | $4,000 | Third-party lab test to verify weatherproofing. $3K-$8K range. |
| Risk assessment documentation (ISO 12100 framework) | $8,000 | Hire safety consultant to produce formal risk assessment. Required by insurance. |
| Product liability insurance (annual premium, pre-pilot) | $8,000 | Estimated for small fleet, pre-revenue startup. $5K-$15K range for $1M-$2M coverage. |
| **Total minimum certifications** | **$33,000** | Full certification ($195K-$380K) comes AFTER successful pilot, funded by Series A. |

### 1.6 Phase 1 Summary: Pre-Pilot Development

| Line Item | Scrappy Founder | Professional Team |
|-----------|----------------|-------------------|
| Prototype iterations (3 rounds) | $21,274 | $21,274 |
| Engineering (6 months) | $57,000 | $228,000 |
| Workspace + tooling | $5,400 | $25,200 |
| ML model training | $485 | $485 |
| Minimum certifications + insurance | $33,000 | $33,000 |
| **Subtotal** | **$117,159** | **$307,959** |
| **Contingency (20%)** | **$23,432** | **$61,592** |
| **Phase 1 Total** | **$140,591** | **$369,551** |

---

## Phase 2: Pilot Production (Month 6-8)

Build 10 pilot units based on the pre-pilot design.

### 2.1 Unit Production Cost

| Item | Per Unit | 10 Units | Notes |
|------|----------|----------|-------|
| BOM at 10-unit pricing | $8,799 | $87,990 | Per BOM research: actuators $4,800, gripper $600, perception $470, compute $249, power $380, bin $160, comms $65, PCB $180, frame $975, wiring $120, 10% contingency |
| Assembly labor (30 hrs/unit @ $50/hr) | $1,500 | $15,000 | Learning curve from prototype means faster assembly. Professional scenario: hired assembler. Scrappy: founder does it. |
| QA/testing per unit (8 hrs @ $50/hr) | $400 | $4,000 | Walk test, sensor calibration, gripper test, weatherproof verification, burn-in test (4hr runtime) |
| **Unit production subtotal** | **$10,699** | **$106,990** | |

### 2.2 Assembly Labor -- Scrappy vs. Professional

| Scenario | Assembly Cost (10 units) | Notes |
|----------|------------------------|-------|
| Scrappy Founder (founder does assembly) | $0 (sweat equity) | Founder + ML engineer assemble all 10 units over 6-8 weeks. Realistic if founder has hardware background. |
| Professional (hired assembler/technician) | $15,000 | 300 hrs total across 10 units |

### 2.3 Additional Production Costs

| Item | Cost | Notes |
|------|------|-------|
| Spare parts inventory | $5,000 | 2x spare actuators ($1,000), 5x spare gripper fingers ($75), spare PCBs ($400), spare battery ($380), misc connectors/cables ($500), spare sensors ($400), spare foot pads ($100), misc ($1,145) |
| Packaging materials (10 units) | $1,500 | Custom foam inserts, heavy-duty cases for shipping. $150/unit. |
| Shipping (to customer site) | $3,000 | Freight shipping, 10 units ~30kg each, domestic US. $300/unit for padded crate shipping. |
| Charging docks (10 units) | $1,500 | Per BOM research: dock BOM $80-$150 each. At 10 units: ~$120 each for pogo pin charger + alignment ramp + enclosure. Plus one extra dock as spare. |

### 2.4 Phase 2 Summary: Pilot Production

| Line Item | Scrappy Founder | Professional Team |
|-----------|----------------|-------------------|
| 10x BOM | $87,990 | $87,990 |
| Assembly labor (10 units) | $0 (sweat equity) | $15,000 |
| QA/testing | $4,000 | $4,000 |
| Spare parts inventory | $5,000 | $5,000 |
| Packaging | $1,500 | $1,500 |
| Shipping | $3,000 | $3,000 |
| Charging docks (10 + 1 spare) | $1,500 | $1,500 |
| **Subtotal** | **$102,990** | **$117,990** |
| **Contingency (15%)** | **$15,449** | **$17,699** |
| **Phase 2 Total** | **$118,439** | **$135,689** |

---

## Phase 3: Pilot Deployment (Month 8-10)

Deploy 10 units at the customer's site (assumed: a US city park or downtown corridor).

### 3.1 Deployment Team

| Item | Scrappy Founder | Professional Team | Notes |
|------|----------------|-------------------|-------|
| Travel (2 engineers, 2 weeks) | $3,000 | $5,000 | Scrappy: founder + ML engineer, budget flights + Airbnb. Professional: 2 engineers, standard travel. |
| Per diem / meals (2 people, 14 days) | $1,400 | $2,100 | Scrappy: $50/person/day. Pro: $75/person/day. |
| Rental vehicle (2 weeks) | $1,200 | $1,200 | Need a van/truck for equipment and robots |
| Local supplies and tools | $500 | $500 | Extension cords, zip ties, markers, tape, misc on-site needs |

### 3.2 Charging Dock Installation

| Item | Cost | Notes |
|------|------|-------|
| Site survey and planning (pre-trip) | $0 | Done remotely via satellite imagery + customer photos |
| Electrical hookup for 10 docks | $3,000 | Electrician to install outdoor GFCI outlets at dock locations. Assumes some outlets exist. $300/dock average for simple outdoor electrical. |
| Dock mounting/anchoring | $500 | Concrete anchors or weighted bases. $50/dock. |
| Weatherproofing (canopy/shelter for docks) | $2,000 | Simple polycarbonate rain shelters. $200/dock. |
| **Total dock installation** | **$5,500** | |

### 3.3 On-Site Setup and Calibration

| Item | Cost | Notes |
|------|------|-------|
| Robot unpacking and assembly (if shipped partially disassembled) | Included in deployment labor | ~2 hrs per unit |
| GPS/RTK base station setup | $500 | RTK correction station for centimeter-level positioning. One per deployment site. |
| Map creation and geofencing | $0 | Done using robot's own sensors + fleet management software. Engineering time only. |
| Calibration and testing (all 10 units) | $0 | Engineering time during deployment trip |
| **Total setup** | **$500** | |

### 3.4 Customer Training

| Item | Cost | Notes |
|------|------|-------|
| Training materials (documentation, video) | $0 | Created during development phase |
| On-site training sessions (2 sessions) | $0 | Delivered by deployment team during trip |
| Dashboard access setup | $0 | Cloud software, provisioned remotely |
| **Total training** | **$0** | No hard cost -- engineering time only |

### 3.5 Phase 3 Summary: Pilot Deployment

| Line Item | Scrappy Founder | Professional Team |
|-----------|----------------|-------------------|
| Travel + per diem | $4,400 | $7,100 |
| Rental vehicle | $1,200 | $1,200 |
| Local supplies | $500 | $500 |
| Dock installation | $5,500 | $5,500 |
| RTK base station | $500 | $500 |
| **Subtotal** | **$12,100** | **$14,800** |
| **Contingency (15%)** | **$1,815** | **$2,220** |
| **Phase 3 Total** | **$13,915** | **$17,020** |

---

## Phase 4: Pilot Operations (Month 10-16, 6-Month Pilot)

### 4.1 Monthly Operating Costs

| Item | Monthly Cost | 6-Month Total | Notes |
|------|-------------|---------------|-------|
| **Cellular data (10 robots)** | $150 | $900 | IoT data plan, ~$15/robot/month (T-Mobile IoT or Hologram) |
| **Cloud infrastructure** | $200 | $1,200 | Fleet management dashboard, data storage, monitoring (Cloudflare R2 + small VPS) |
| **Remote monitoring labor** | Variable | Variable | See scenario split below |
| **Maintenance visits** | Variable | Variable | See scenario split below |
| **Replacement parts** | $400 | $2,400 | Budget for ongoing wear: gripper fingers ($150/mo), actuator bearings, foot pads, misc |
| **Product liability insurance** | $700 | $4,200 | Ongoing annual premium, allocated monthly. Higher during active deployment. |
| **Software licenses** | $100 | $600 | ROS2 is free, but budget for monitoring tools, map services, etc. |
| **ML model updates (retraining)** | $50 | $300 | 1-2 retraining runs during pilot based on field data |

### 4.2 Remote Monitoring -- Scrappy vs. Professional

| Scenario | Approach | Monthly Cost | 6-Month Total |
|----------|----------|-------------|---------------|
| Scrappy Founder | Founder monitors dashboard evenings/weekends; ML engineer monitors during work hours; alerts to phone | $0 (sweat equity) | $0 |
| Professional Team | Dedicated operations engineer (part-time, 20 hrs/week) | $4,000/mo | $24,000 |

### 4.3 Maintenance Visits

| Scenario | Approach | Monthly Cost | 6-Month Total |
|----------|----------|-------------|---------------|
| Scrappy Founder | Monthly 2-day trip by founder (if not local). Flights $300 + hotel $200 + per diem $100 | $600/mo | $3,600 |
| Professional Team | Hire local technician (part-time contract, 10 hrs/week @ $40/hr) | $1,600/mo | $9,600 |
| Either (if local deployment) | On-site visits 2x/week, 2 hrs each | $0 (sweat equity) or $1,600/mo | $0 or $9,600 |

### 4.4 Phase 4 Summary: Pilot Operations (6 Months)

| Line Item | Scrappy Founder | Professional Team |
|-----------|----------------|-------------------|
| Cellular data (10 robots, 6 mo) | $900 | $900 |
| Cloud infrastructure (6 mo) | $1,200 | $1,200 |
| Remote monitoring (6 mo) | $0 (sweat equity) | $24,000 |
| Maintenance visits (6 mo) | $3,600 | $9,600 |
| Replacement parts (6 mo) | $2,400 | $2,400 |
| Insurance (6 mo) | $4,200 | $4,200 |
| Software licenses (6 mo) | $600 | $600 |
| ML model updates (6 mo) | $300 | $300 |
| **Subtotal** | **$13,200** | **$43,200** |
| **Contingency (15%)** | **$1,980** | **$6,480** |
| **Phase 4 Total** | **$15,180** | **$49,680** |

---

## Revenue Model

### 5.1 Pilot Contract Structure

10-unit, 6-month pilot at discounted RaaS rate.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Units deployed | 10 | |
| Pilot duration | 6 months | |
| Monthly rate per unit | $1,200 | 40% discount from standard $2,000/mo rate |
| Total contract value | **$72,000** | 10 x $1,200 x 6 |
| Upfront payment (20%) | **$14,400** | Collected at contract signing (Month 8-9) |
| Remaining balance | $57,600 | |
| Monthly payments | **$9,600/month** | $57,600 / 6 months |

### 5.2 Payment Schedule

| Month | Event | Cash In |
|-------|-------|---------|
| Month 9 | Contract signed, 20% upfront payment | $14,400 |
| Month 10 | Pilot month 1, first monthly payment | $9,600 |
| Month 11 | Pilot month 2 | $9,600 |
| Month 12 | Pilot month 3 | $9,600 |
| Month 13 | Pilot month 4 | $9,600 |
| Month 14 | Pilot month 5 | $9,600 |
| Month 15 | Pilot month 6 (final) | $9,600 |
| **Total** | | **$72,000** |

---

## Summary: The CEO's One-Pager

### Total Investment to Reach Pilot

| Phase | Scrappy Founder | Professional Team |
|-------|----------------|-------------------|
| Pre-pilot development (Month 1-6) | $140,591 | $369,551 |
| Pilot production -- 10 units (Month 6-8) | $118,439 | $135,689 |
| Deployment (Month 8-10) | $13,915 | $17,020 |
| Operations -- 6 months (Month 10-16) | $15,180 | $49,680 |
| **Total cost to us** | **$288,125** | **$571,940** |

### Revenue from Pilot

| Item | Amount |
|------|--------|
| Total pilot contract value | $72,000 |
| 20% upfront payment (Month 9) | $14,400 |
| Monthly payments (Month 10-15) | $9,600/month |

### Key Metrics

| Metric | Scrappy Founder | Professional Team |
|--------|----------------|-------------------|
| Total investment required | $288,125 | $571,940 |
| Total pilot revenue | $72,000 | $72,000 |
| **Pilot P&L (loss)** | **-$216,125** | **-$499,940** |
| Cash needed before first payment (Month 1-9) | $273,000 | $522,000 |
| Cash needed before pilot revenue covers monthly ops | $275,500 | $530,000 |
| Break-even on this pilot alone | **Never** | **Never** |
| Revenue as % of cost | 25% | 12.6% |

**This pilot will NOT be profitable. That is expected and normal.** The first pilot is about proof, learning, and building a reference customer -- not profit. Every robotics company (Starship, Nuro, Avidbots) lost significant money on early deployments.

### What the Pilot IS Worth (Beyond Revenue)

- A live customer reference and video case study for fundraising
- Real-world performance data to improve the ML model and hardware reliability
- Proof that the business model works (customer willingness to pay)
- Foundation for Series A pitch: "We deployed 10 units, collected X thousand pieces of litter, achieved Y% uptime, and the customer wants to expand"
- De-risked path to full certification and production scale

### What the CEO Says to Customers

"Our pilot program puts 10 autonomous litter-collecting robots in your park for $1,200 per unit per month over 6 months -- a total investment of $72,000 with 20% paid upfront. That replaces the equivalent of 15-25 FTEs of manual litter collection labor at a fraction of the cost. You will see measurable cleanliness improvements within the first month, with full performance data on your dashboard in real time."

### What the CEO Says to Investors

"We need $350,000 to build and deploy our first 10-unit commercial pilot with a US municipality. The pilot generates $72,000 in revenue and -- more importantly -- gives us a live customer reference, real-world performance data, and proof of product-market fit. Our path to profitability is scaling to 100+ units where hardware costs drop 30%, operations are amortized across the fleet, and each unit generates $24,000/year in recurring RaaS revenue at 55%+ gross margins."

---

## Cash Flow Timeline

### Monthly Cash Flow (Scrappy Founder Scenario)

| Month | Phase | Cash Out | Cash In | Cumulative |
|-------|-------|----------|---------|------------|
| 1 | Pre-pilot dev | -$25,000 | $0 | -$25,000 |
| 2 | Pre-pilot dev | -$25,000 | $0 | -$50,000 |
| 3 | Pre-pilot dev | -$20,000 | $0 | -$70,000 |
| 4 | Pre-pilot dev | -$18,000 | $0 | -$88,000 |
| 5 | Pre-pilot dev | -$15,000 | $0 | -$103,000 |
| 6 | Pre-pilot dev + production start | -$38,000 | $0 | -$141,000 |
| 7 | Pilot production | -$55,000 | $0 | -$196,000 |
| 8 | Pilot production + deployment prep | -$40,000 | $0 | -$236,000 |
| 9 | Deployment + contract signing | -$14,000 | +$14,400 | -$235,600 |
| 10 | Operations month 1 | -$2,500 | +$9,600 | -$228,500 |
| 11 | Operations month 2 | -$2,500 | +$9,600 | -$221,400 |
| 12 | Operations month 3 | -$2,500 | +$9,600 | -$214,300 |
| 13 | Operations month 4 | -$2,500 | +$9,600 | -$207,200 |
| 14 | Operations month 5 | -$2,500 | +$9,600 | -$200,100 |
| 15 | Operations month 6 (pilot ends) | -$2,500 | +$9,600 | -$193,000 |

**Peak cash requirement: ~$236,000 (Month 8)** -- this is the maximum amount of money that must be in the bank before any revenue comes in.

**Post-pilot cumulative loss: ~$193,000** -- the robots still exist and can generate revenue from a contract extension or new customer.

### Monthly Cash Flow (Professional Team Scenario)

| Month | Phase | Cash Out | Cash In | Cumulative |
|-------|-------|----------|---------|------------|
| 1 | Pre-pilot dev | -$60,000 | $0 | -$60,000 |
| 2 | Pre-pilot dev | -$55,000 | $0 | -$115,000 |
| 3 | Pre-pilot dev | -$50,000 | $0 | -$165,000 |
| 4 | Pre-pilot dev | -$48,000 | $0 | -$213,000 |
| 5 | Pre-pilot dev | -$45,000 | $0 | -$258,000 |
| 6 | Pre-pilot dev + production start | -$75,000 | $0 | -$333,000 |
| 7 | Pilot production | -$80,000 | $0 | -$413,000 |
| 8 | Pilot production + deployment prep | -$60,000 | $0 | -$473,000 |
| 9 | Deployment + contract signing | -$25,000 | +$14,400 | -$483,600 |
| 10 | Operations month 1 | -$8,300 | +$9,600 | -$482,300 |
| 11 | Operations month 2 | -$8,300 | +$9,600 | -$481,000 |
| 12 | Operations month 3 | -$8,300 | +$9,600 | -$479,700 |
| 13 | Operations month 4 | -$8,300 | +$9,600 | -$478,400 |
| 14 | Operations month 5 | -$8,300 | +$9,600 | -$477,100 |
| 15 | Operations month 6 (pilot ends) | -$8,300 | +$9,600 | -$475,800 |

**Peak cash requirement: ~$484,000 (Month 9)**

---

## Sensitivity Analysis

### What If We Charge More?

| Monthly Rate/Unit | Total Pilot Revenue | Pilot P&L (Scrappy) | Pilot P&L (Pro) |
|-------------------|--------------------|-----------------------|------------------|
| $800/unit (deep discount) | $48,000 | -$240,125 | -$523,940 |
| $1,000/unit | $60,000 | -$228,125 | -$511,940 |
| **$1,200/unit (baseline)** | **$72,000** | **-$216,125** | **-$499,940** |
| $1,500/unit | $90,000 | -$198,125 | -$481,940 |
| $2,000/unit (standard rate) | $120,000 | -$168,125 | -$451,940 |

Even at full $2,000/month standard pricing, the first pilot is unprofitable. The pricing decision is about customer acquisition, not profit maximization. The 40% pilot discount ($1,200/mo) is the right call -- it lowers the barrier for the city and the incremental revenue from charging $2,000 does not change the fundamental economics.

### What If We Deploy Fewer Units?

| Units | Revenue ($1,200/mo, 6mo) | Pilot P&L (Scrappy) | Cash Needed |
|-------|-------------------------|----------------------|-------------|
| 3 units | $21,600 | -$204,200 | $175,000 |
| 5 units | $36,000 | -$208,500 | $200,000 |
| **10 units (baseline)** | **$72,000** | **-$216,125** | **$236,000** |
| 15 units | $108,000 | -$233,000 | $290,000 |

Smaller pilots cost less in total but generate even less revenue per dollar spent. The 10-unit pilot is the sweet spot: large enough for meaningful data, small enough to keep costs manageable.

### What If Something Breaks?

| Scenario | Additional Cost | Impact |
|----------|----------------|--------|
| 1 actuator failure per month (worst case) | +$3,000 over 6 months | Manageable with spare inventory |
| 2 complete unit failures during pilot | +$17,600 (replace from spares + emergency rebuild) | Painful but survivable. This is why we carry spares. |
| Battery degradation requires 3 replacements | +$1,140 | Minor |
| Gripper redesign needed mid-pilot | +$3,500 (new mold + parts for 10 units) | Expected possibility. Budget is in contingency. |

---

## Key Assumptions and Honest Disclaimers

### Where These Numbers Could Be Wrong

1. **Prototype iterations might take more than 3 rounds.** Hardware is unpredictable. Budget for 4-5 iterations in fundraising conversations, even though we model 3.

2. **Assembly time per unit could be higher.** The 30-hour estimate for production units assumes the design is finalized and documented. If the design is still changing during production, expect 40-50 hours.

3. **Certification costs could surprise us.** The $33K "minimum certification" is for a cooperative municipality that does not require full UL/ISO certification for a pilot. Some cities might require more. This could add $20K-$80K.

4. **Actuator failures are the biggest unknown.** CubeMars AK70-10 actuators are well-regarded in research but have not been proven in sustained outdoor commercial use. Failure rates could be higher than expected.

5. **Customer acquisition timeline is uncertain.** We model deployment starting at Month 8-10, but finding and signing a pilot customer could take 6-18 months (per market research). The CEO should start sales conversations at Month 1, not Month 8.

6. **Founder burnout is a real cost.** The "scrappy founder" scenario assumes the founder works 60-80 hour weeks for 15 months with no salary. This is not sustainable long-term and not reflected in the financial model.

### What This Model Does NOT Include

- **Full safety certification** ($195K-$380K) -- deferred to post-pilot, funded by Series A
- **Full-time sales/marketing** -- assumes founder does all sales
- **Legal costs** (incorporation, contracts, IP) -- estimated $5K-$15K additional
- **Office/facilities beyond basic workspace** -- no fancy office
- **Travel for sales meetings** during pre-pilot phase -- estimated $3K-$8K additional
- **Taxes, payroll taxes, benefits** for professional team -- would add ~25-35% to labor costs
- **Equipment depreciation** -- robots are treated as a sunk cost for this analysis
- **Opportunity cost of founder's time** -- the biggest hidden cost

### The Honest Bottom Line

**Scrappy Founder path:** You need roughly **$250,000-$300,000** to get through the first pilot. This is a realistic pre-seed/seed fundraise for a hardware robotics startup with a working prototype and a signed pilot customer.

**Professional Team path:** You need roughly **$500,000-$600,000**. This is a larger seed round and requires either strong investor conviction or significant grant funding.

**In either case, the pilot itself does not pay for the development that preceded it.** The pilot is a customer development exercise and a fundraising tool. Its purpose is to generate the data, reference, and credibility needed to raise a Series A of $2M-$5M to fund full certification, production scale-up (100 units), and the path to unit-level profitability.

---

## Appendix A: Unit Economics at Scale (Post-Pilot Reference)

For context on the long-term business case (what you tell investors):

| Metric | At 100 Units | At 1,000 Units |
|--------|-------------|----------------|
| BOM per unit | $7,262 | $5,503 |
| Assembly + QA per unit | $1,200 | $600 |
| Total hardware cost per unit | $8,462 | $6,103 |
| Annual RaaS revenue per unit | $24,000 | $24,000 |
| Annual operating cost per unit | $4,000 | $2,500 |
| Annual gross profit per unit | $12,000-$15,000 | $17,000-$19,000 |
| Gross margin | 50-63% | 71-79% |
| Hardware payback period | 5-7 months | 3-4 months |

**The business becomes profitable at scale. The pilot proves it can get there.**

## Appendix B: Data Sources

All cost estimates in this model are derived from the following internal research documents:
- `docs/hardware-bom-research.md` -- Hardware BOM and supplier pricing
- `docs/ml-compute-costs.md` -- ML training and inference costs
- `docs/market-research.md` -- Market sizing, pricing, and competitive analysis
- `docs/product-spec-research.md` -- Product specifications and certification requirements

Engineering labor rates based on Bay Area market rates for robotics engineers (Glassdoor, Levels.fyi, 2026 data). Insurance estimates based on broker conversations and industry benchmarks for autonomous systems startups.

---

*This model uses conservative estimates throughout: costs are rounded up, revenue is rounded down, and contingency buffers of 15-20% are applied to every phase. The purpose is to give the CEO and investors a realistic, worst-case-aware picture of the financial requirements for a first pilot deployment.*
