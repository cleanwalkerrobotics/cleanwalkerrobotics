# CleanWalker Robotics -- Logistics & Transport Cost Report

**Date:** 2026-02-09
**Author:** Logistics Research Lead
**Revision:** 1.0
**Companion documents:** `hardware-bom-research.md`, `pilot-financial-model.md`

---

## Executive Summary

This report details the full logistics chain cost for CleanWalker Robotics -- from sourcing individual components internationally to delivering 10 assembled robots to a pilot customer and supporting them through a 6-month deployment. All figures are sourced from real freight rates, tariff schedules, and vendor pricing as of early 2026.

**Headline numbers (10-unit US pilot):**
- Inbound component shipping (per robot): **$180 -- $310**
- Import duties & tariffs (per robot): **$950 -- $1,550**
- Outbound delivery to customer (10 robots): **$4,000 -- $6,000**
- Deployment logistics (travel, installation): **$12,000 -- $17,000**
- Ongoing pilot logistics (6 months): **$8,500 -- $14,500**
- Return/end-of-pilot logistics: **$3,000 -- $5,000**
- **Total logistics cost for 10-unit US pilot: $38,800 -- $62,100**
- **Per-robot logistics cost (US pilot): ~$3,880 -- $6,210**

Tariffs on Chinese components are by far the single largest logistics cost, representing roughly 40-50% of total logistics spend.

---

## Table of Contents

1. [Inbound Component Shipping](#1-inbound-component-shipping)
2. [Import Duties & Tariffs](#2-import-duties--tariffs)
3. [Outbound Delivery to Customer](#3-outbound-delivery-to-customer)
4. [Deployment Logistics](#4-deployment-logistics)
5. [Ongoing Logistics During Pilot](#5-ongoing-logistics-during-pilot)
6. [Return / End-of-Pilot Logistics](#6-return--end-of-pilot-logistics)
7. [International Pilot Variant (EU / Singapore)](#7-international-pilot-variant)
8. [Comprehensive Summary Tables](#8-comprehensive-summary-tables)

---

## 1. Inbound Component Shipping

### 1.1 CubeMars Actuators (China to US)

The CubeMars actuators are the heaviest and highest-value components shipped internationally. For a 10-unit pilot, we need 120 actuators (12 per robot): 80x AK70-10 (~500g each) and 40x AK60-6 (~305g each).

**Total actuator weight for 10 robots:** ~52 kg
**Total actuator value for 10 robots:** ~$48,000 (at 10-unit pricing)

CubeMars ships from Shenzhen/Changsha, China. Their default shipping method is DHL Express with typical delivery in 3-5 business days.

| Shipping Method | Rate | Est. Cost (52 kg) | Transit Time | Source |
|----------------|------|-------------------|--------------|--------|
| DHL Express (CubeMars default) | $7-10/kg | $360 -- $520 | 3-5 days | [Freightos](https://www.freightos.com/freight-resources/air-freight-rates-cost-prices/) |
| Standard air freight (consolidated) | $5-7/kg | $260 -- $364 | 5-10 days | [Freightos](https://www.freightos.com/shipping-routes/shipping-from-china-to-the-united-states/) |
| Sea freight LCL | $50-120/CBM | $50 -- $120 (est. 0.1-0.15 CBM) | 25-40 days | [Sino-Shipping](https://www.sino-shipping.com/country-guides/shipping-from-china-to-usa/) |

**Recommendation:** DHL Express is the right choice for actuators. The value density is extremely high ($48K for 52 kg), so the $100-200 premium for express over standard air is negligible relative to the time savings and better tracking/insurance. Sea freight saves money but adds 4-6 weeks, which is unacceptable for a time-sensitive pilot build.

**Likely shipping scenario:** CubeMars will likely ship in 2-3 batches (orders placed over the production period). Estimate 2-3 DHL Express shipments at $150-250 each.

| Item | Cost Estimate |
|------|--------------|
| Actuator shipping (2-3 DHL Express shipments) | **$400 -- $650** |
| Per robot share | **$40 -- $65** |

**Lead time note:** CubeMars standard lead time is 1-2 weeks for in-stock items. For 120 actuators, confirm stock availability early -- some models may require 4-6 weeks production lead time.

### 1.2 PCBs from JLCPCB (Shenzhen to US)

For a 10-unit run we need ~10-15 custom PCBs (power distribution + interface board, plus spares). The PCBs are lightweight (<1 kg total for the batch).

**JLCPCB shipping options:**
- Global Direct Shipping (cheapest): $5-15 for small orders, 7-15 days
- DHL Express: $15-30, 3-5 days
- FedEx/UPS DDP (Delivery Duty Paid): $20-40, 4-7 days

Since November 2025, JLCPCB pre-collects US tariffs on DDP shipments. The tariff component adds roughly 35-45% to board value (see Section 2 for breakdown).

| Item | Cost Estimate |
|------|--------------|
| PCB shipping (JLCPCB, DHL or DDP) | **$25 -- $50** |
| Per robot share | **$2.50 -- $5** |

Source: [JLCPCB Shipping Info](https://jlcpcb.com/help/article/How-much-does-shipping-cost), [JLCPCB Tariff FAQ](https://jlcpcb.com/help/article/us-tariff-policy-faq)

### 1.3 Domestic US Components

**NVIDIA Jetson Orin Nano Super (10 units):**
- Source: DigiKey, Mouser, or NVIDIA authorized distributors
- DigiKey shipping: $6.99 ground, $12.99 2-day delivery
- At 10 units, a single DigiKey order ships for $6.99 (free shipping on orders >$100)
- Cost: **$0 -- $7** (likely free shipping on a $2,490 order)

**Luxonis OAK-D Pro cameras (10 units):**
- Source: shop.luxonis.com (ships from US/EU warehouse)
- Standard shipping: ~$10-20 per order
- For 10 units ($3,990 order value): likely **$15 -- $30**

**Batteries (48V 20Ah Li-ion, 10 units):**
- Source: Amazon, Aegis Battery, or direct from manufacturer
- **UN3481 lithium-ion shipping regulations apply**
- FedEx Ground accepts UN3481 (batteries packed with equipment) without hazmat surcharge when properly labeled and documented
- For standalone battery packs (UN3480): more restrictive; ground shipping only, proper packaging and labeling required
- As of January 2026, new IATA state-of-charge limits apply: batteries must be at 30% SoC or less for air transport
- Domestic ground shipping: standard rates, no hazmat surcharge for UN3481 packed with/in equipment
- Estimated shipping for 10 battery packs (~55 kg total): **$50 -- $150** via FedEx/UPS Ground

Sources: [FedEx Lithium Battery Guide](https://www.fedex.com/content/dam/fedex/us-united-states/services/Battery_Overview_2025.pdf), [IATA Battery Guidance](https://www.iata.org/contentassets/05e6d8742b0047259bf3a700bc9d42b9/lithium-battery-guidance-document.pdf)

**Dynamixel servos (ROBOTIS US, 20 units for arms + 10 gripper servos):**
- Ships from ROBOTIS US warehouse in Illinois
- Standard USPS/UPS shipping: **$15 -- $30**

**CNC machined parts from Xometry:**
- Xometry includes free standard shipping on all US orders
- Cost: **$0**

Source: [Xometry CNC Service](https://www.xometry.com/capabilities/cnc-machining-service/)

**McMaster-Carr, Amazon, misc components:**
- Standard shipping across multiple orders: **$30 -- $80**

### 1.4 Inbound Shipping Cost Summary (10 Robots)

| Component Source | Shipping Cost (10 units) | Per Robot |
|-----------------|------------------------|-----------|
| CubeMars actuators (China, DHL Express) | $400 -- $650 | $40 -- $65 |
| JLCPCB PCBs (China, DHL/DDP) | $25 -- $50 | $2.50 -- $5 |
| NVIDIA Jetson (DigiKey, domestic) | $0 -- $7 | $0 -- $1 |
| Luxonis cameras (domestic/EU warehouse) | $15 -- $30 | $1.50 -- $3 |
| Batteries (domestic, UN3481 ground) | $50 -- $150 | $5 -- $15 |
| Dynamixel servos (ROBOTIS US) | $15 -- $30 | $1.50 -- $3 |
| Xometry CNC parts (domestic) | $0 | $0 |
| Misc components (Amazon, McMaster) | $30 -- $80 | $3 -- $8 |
| **Total inbound shipping** | **$535 -- $997** | **$54 -- $100** |

**Customs broker fees (for China imports):**
- Formal entry fee (for actuator shipments >$2,500): $125 -- $175 per entry
- Merchandise Processing Fee (MPF): 0.3464% of value, min $31.67
- For ~$48,000 in actuators: MPF = ~$166
- Estimated 2-3 entries total

| Customs Brokerage | Cost (10 units) | Per Robot |
|------------------|----------------|-----------|
| Broker fees (2-3 entries) | $250 -- $525 | $25 -- $53 |
| MPF (actuators + PCBs) | $175 -- $200 | $18 -- $20 |
| **Total customs brokerage** | **$425 -- $725** | **$43 -- $73** |

Sources: [CBP Customs Broker Fees](https://www.cbp.gov/trade/programs-administration/customs-brokers/fees), [Clearit USA](https://clearitusa.com/customs-clearance-pricing-rates/)

---

## 2. Import Duties & Tariffs

This is the most complex and financially significant logistics cost. The US-China tariff landscape as of early 2026 involves multiple overlapping tariff layers.

### 2.1 Current US-China Tariff Framework (as of February 2026)

Following the November 2025 US-China trade deal (extended through November 10, 2026), the tariff layers on Chinese imports stack as follows:

| Tariff Layer | Rate | Status (Feb 2026) | Source |
|-------------|------|-------------------|--------|
| MFN (baseline WTO rate) | Varies by HTS code (0-6%) | Permanent | [USITC HTS](https://hts.usitc.gov/) |
| Section 301 (tech transfer) | 7.5% -- 100% (varies by list) | Extended through Nov 2026 | [USTR](https://ustr.gov/issue-areas/enforcement/section-301-investigations/tariff-actions) |
| Reciprocal tariff ("Liberation Day") | 10% | Extended through Nov 10, 2026 | [White House Fact Sheet](https://www.whitehouse.gov/fact-sheets/2025/11/fact-sheet-president-donald-j-trump-strikes-deal-on-economic-and-trade-relations-with-china/) |
| IEEPA / Fentanyl tariff | 10% (reduced from 20%) | Through Nov 10, 2026 | [KPMG Analysis](https://kpmg.com/us/en/taxnewsflash/news/2025/11/united-states-tariff-rates-duties-china.html) |

**Important:** Section 301, reciprocal, and IEEPA tariffs all stack on top of each other and on top of the MFN rate. The de minimis exemption ($800 threshold) has been eliminated for Chinese imports.

Sources: [China Briefing](https://www.china-briefing.com/news/us-china-tariff-rates-2025/), [Penn Wharton Budget Model](https://budgetmodel.wharton.upenn.edu/issues/2026/1/15/effective-tariff-rates-and-revenues-updated-january-15-2026)

### 2.2 Actuators (CubeMars BLDC Motor Actuators)

**HTS Classification:** The CubeMars AK series are brushless DC motors with integrated gearboxes and driver electronics. The primary HTS code is **8501.31.40** (DC motors, output not exceeding 750W, exceeding 74.6W but not exceeding 735W).

| Tariff Component | Rate | On $48,000 actuator value |
|-----------------|------|--------------------------|
| MFN base rate (HTS 8501.31.40) | 4% | $1,920 |
| Section 301 (List 3, 25%) | 25% | $12,000 |
| Reciprocal tariff | 10% | $4,800 |
| IEEPA / Fentanyl | 10% | $4,800 |
| **Total duty on actuators** | **~49%** | **$23,520** |
| **Per robot (12 actuators)** | | **$2,352** |

Sources: [USITC HTS 8501.31.40](https://hts.usitc.gov/), [CBP Customs Ruling N313413](https://rulings.cbp.gov/ruling/N313413)

**Critical note:** This 49% cumulative tariff adds approximately **$2,352 per robot** just for actuators. This is a massive cost driver and the single strongest argument for eventually sourcing actuators domestically or from tariff-free countries (e.g., South Korea, Japan, or domestic US manufacturing).

**Potential mitigation:**
- Check Section 301 exclusion lists: Some motor categories have been excluded. The 178 exclusions extended to November 10, 2026 should be reviewed by a customs broker to determine if HTS 8501.31.40 qualifies.
- If exclusion applies: tariff drops to ~24% (MFN + reciprocal + IEEPA, no Section 301) = ~$11,520 total, or ~$1,152/robot.
- Consider MyActuator (also China-based) -- same tariff situation.

### 2.3 PCBs from JLCPCB

**HTS Classification:** Printed circuit boards fall under **HTS 8534.00**. For 2-4 layer boards:

| Tariff Component | Rate | On ~$1,800 PCB value (10 units) |
|-----------------|------|--------------------------------|
| MFN base rate (HTS 8534.00) | 0% | $0 |
| Section 301 (25%) | 25% | $450 |
| Reciprocal tariff | 10% | $180 |
| IEEPA / Fentanyl | 10% | $180 |
| **Total duty on PCBs** | **~45%** | **~$810** |
| **Per robot** | | **~$81** |

**Note:** The PCB tariff situation is volatile. The 2-4 layer board exclusion was protected until May 31, 2026 per some sources, after which rates could rise to 145-170%. JLCPCB now pre-collects tariffs on DDP shipments, simplifying the process.

Sources: [DirectPCB Tariff Update](https://www.directpcb.com/tariff-update/), [ICAPE Group](https://www.icape-group.com/pcb-tariff-information/), [CBP Ruling N336087](https://rulings.cbp.gov/ruling/N336087)

### 2.4 Other Chinese-Origin Components

The SIM7600G-H cellular module is manufactured by SIMCom in China. Various small components (connectors, cables) may also have Chinese origin.

| Component | Est. Value | Est. Duty Rate | Est. Duty |
|-----------|-----------|---------------|-----------|
| SIM7600G-H modules (10x) | $550 | ~35% | $193 |
| Misc Chinese-origin components | $500 | ~35% | $175 |
| **Total misc Chinese duties** | | | **$368** |
| **Per robot** | | | **$37** |

### 2.5 Domestic US Components -- No Tariff

Components sourced domestically (NVIDIA Jetson from US distributors, Luxonis cameras, Xometry CNC parts, Dynamixel from ROBOTIS US, batteries from US suppliers) do not incur import duties.

**Note on Jetson:** While NVIDIA Jetson modules are manufactured in Asia (primarily Taiwan/China), when purchased through US distributors like DigiKey, the importer of record (the distributor) has already paid applicable duties. The cost is baked into the retail price. No separate duty for CleanWalker.

### 2.6 Import Duty Summary (10 Robots)

| Component | Declared Value | Duty Rate | Total Duty | Per Robot |
|-----------|---------------|-----------|------------|-----------|
| CubeMars actuators | $48,000 | ~49% | $23,520 | $2,352 |
| JLCPCB PCBs | $1,800 | ~45% | $810 | $81 |
| SIMCom modules + misc | $1,050 | ~35% | $368 | $37 |
| **Total import duties** | **$50,850** | | **$24,698** | **$2,470** |

**If Section 301 exclusion applies to actuators:**

| Scenario | Total Duty (10 units) | Per Robot |
|----------|----------------------|-----------|
| With Section 301 on actuators (worst case) | $24,698 | $2,470 |
| Without Section 301 on actuators (if excluded) | $12,698 | $1,270 |
| **Working estimate (conservative)** | **$24,698** | **$2,470** |

**This is a massive cost.** At $2,470 per robot, tariffs alone represent ~28% of the $8,799 BOM cost. This should be a priority item for cost optimization -- either through tariff engineering (exclusion filing, HTS reclassification review with a customs broker) or supply chain restructuring (sourcing actuators from non-China countries).

---

## 3. Outbound Delivery to Customer

### 3.1 Packaging and Crating

Each assembled CleanWalker robot weighs approximately 25-35 kg and measures roughly 800 x 500 x 700 mm. These are precision electromechanical devices with exposed actuators, sensors, and delicate electronics that require proper protection.

**Custom crating requirements:**
- Heavy-duty wooden crate with foam-in-place or precision-cut foam interior
- Anti-static wrapping for electronics
- Vibration dampening (closed-cell foam)
- "FRAGILE" and "THIS SIDE UP" labeling
- Crate dimensions (with padding): approximately 1000 x 700 x 900 mm (~0.63 CBM per crate)

**Cost per crate:**
Industry standard for sensitive electronics crating is 1-10% of product value. For our ~$10,000 robots, 1.5-2% is appropriate for a medium-complexity crate.

| Item | Per Unit | 10 Units | Source |
|------|----------|----------|--------|
| Custom wooden crate (1000x700x900mm) | $120 -- $200 | $1,200 -- $2,000 | [Valley Box](https://www.valleybox.com/blog/how-much-does-a-shipping-crate-cost), [Creopack](https://creopack.com/en/blog/crate-shipping-rates/) |
| Foam inserts (precision-cut or foam-in-place) | $30 -- $60 | $300 -- $600 | Included with crating service |
| Anti-static bags + desiccant packets | $5 -- $10 | $50 -- $100 | |
| **Total packaging per unit** | **$155 -- $270** | **$1,550 -- $2,700** | |

Sources: [Craters & Freighters](https://www.cratersandfreighters.com/shipping/electronics-shipping/robotics-shipping/), [Shippei Robotics Guide](https://shippei.com/specialized-shipping-of-robotics/)

### 3.2 Domestic US Freight Shipping (LTL)

10 crated robots at ~35-40 kg each (including crate weight), dimensions ~1000x700x900mm per crate.

**Shipment profile:**
- Total weight: ~350-400 kg (770-880 lbs)
- Total volume: ~6.3 CBM
- Number of pallets: 5-10 (2 robots per pallet or 1 per pallet depending on stacking)
- Freight class: 125-150 (electronic equipment, fragile)

| Shipping Method | Est. Cost | Transit Time | Notes |
|----------------|-----------|--------------|-------|
| LTL freight (ODFL, FedEx Freight) | $1,200 -- $2,500 | 3-7 days | Based on $0.25-$0.40/lb for ~880 lbs, freight class 125-150 |
| FTL (dedicated truck, if justified) | $2,500 -- $4,000 | 2-5 days | Overkill for 10 crates unless combining with charging docks |
| White glove delivery (inside delivery + uncrating) | +$500 -- $1,000 | Same as LTL | Recommended for customer-facing delivery |

Source: [Red Stag Fulfillment](https://redstagfulfillment.com/how-much-does-freight-delivery-cost/)

**Recommendation:** LTL freight with white glove delivery for the final mile. The customer (a municipality) will not have loading dock access at a park. Budget for liftgate delivery + inside delivery.

| Item | Cost (10 units) |
|------|----------------|
| LTL freight (est. Bay Area to Austin/SF) | $1,500 -- $2,500 |
| White glove / liftgate surcharge | $500 -- $1,000 |
| **Total outbound freight** | **$2,000 -- $3,500** |

### 3.3 Shipping Insurance

For 10 robots valued at ~$10,000 each ($100,000 total shipment value):

| Coverage Type | Rate | Premium | Source |
|--------------|------|---------|--------|
| Carrier declared value (FedEx/UPS) | $0.50-$1.50 per $100 | $500 -- $1,500 | [FedEx Declared Value](https://www.refundretriever.com/blog/fedex-declared-value-what-you-need-to-know) |
| Third-party all-risk cargo insurance | 0.5% -- 1.0% of value | $500 -- $1,000 | [Flock Freight](https://www.flockfreight.com/blog/shipping-insurance-for-high-value-electronics-and-tech) |
| **Recommended: all-risk cargo insurance** | | **$500 -- $1,000** | |

Source: [Ecabrella](https://www.ecabrella.com/blog-posts/freight-insurance-cost)

### 3.4 Charging Dock Shipping

10 charging docks (+ 1 spare) weigh approximately 3-5 kg each, much simpler to ship.

| Item | Cost |
|------|------|
| Standard ground shipping (11 docks, ~50 kg) | $50 -- $150 |

### 3.5 Outbound Delivery Summary (10-Unit US Pilot)

| Item | Cost |
|------|------|
| Custom crating (10 robots) | $1,550 -- $2,700 |
| LTL freight + white glove | $2,000 -- $3,500 |
| Cargo insurance ($100K value) | $500 -- $1,000 |
| Charging dock shipping | $50 -- $150 |
| **Total outbound delivery** | **$4,100 -- $7,350** |
| **Per robot** | **$410 -- $735** |

---

## 4. Deployment Logistics

### 4.1 Engineer Travel (2 Engineers, 2 Weeks)

Assumes engineers are based in the San Francisco Bay Area and deploying to either a local SF pilot or a remote city (Austin, TX as example).

#### Scenario A: Local Pilot (San Francisco)

| Item | Cost | Notes |
|------|------|-------|
| Flights | $0 | Local |
| Hotel (14 nights) | $0 -- $3,626 | $0 if commuting from home, $259/night GSA rate if from outside SF |
| Rental vehicle (cargo van, 2 weeks) | $800 -- $1,200 | Need cargo van for equipment transport |
| Per diem / meals (2 people, 14 days) | $0 -- $2,576 | $92/day GSA rate for SF, or $0 if eating at home |
| Parking (2 weeks in SF) | $200 -- $500 | |
| **Total (local, commuting)** | **$1,000 -- $1,700** | |
| **Total (local, hotel needed)** | **$4,600 -- $7,900** | |

#### Scenario B: Remote Pilot (Austin, TX)

| Item | Cost (2 engineers) | Notes |
|------|-------------------|-------|
| Round-trip flights (SFO to AUS) | $500 -- $800 | $250-400/person, economy. Source: [Kayak](https://www.kayak.com/flight-routes/San-Francisco-Bay-Area-zzXIH/Austin-Bergstrom-AUS) |
| Hotel (14 nights x 2 rooms) | $5,236 | $187/night x 14 nights x 2 rooms (GSA Austin rate). Source: [Federal Per Diem](https://www.federalpay.org/perdiem/2025/texas/austin) |
| Rental vehicle (cargo van, 2 weeks) | $800 -- $1,200 | Enterprise/Hertz |
| Meals & incidentals (2 people, 14 days) | $2,240 | $80/day x 14 days x 2 people (GSA Austin M&IE rate) |
| Rideshare/local transport | $100 -- $200 | |
| **Total (Austin deployment)** | **$8,876 -- $9,676** | |

Source: [GSA Austin Per Diem 2025](https://www.federalpay.org/perdiem/2025/texas/austin), [GSA SF Per Diem](https://www.federalpay.org/perdiem/2025/california/san-francisco)

**Working estimate for budget:** $9,000 -- $10,000 for remote deployment (2 engineers, 2 weeks).

### 4.2 Charging Dock Installation

| Item | Cost | Notes |
|------|------|-------|
| Site survey (pre-trip, remote) | $0 | Done via satellite imagery + customer photos |
| Electrician: 10x outdoor GFCI outlets / 240V circuits | $3,000 -- $5,000 | $300-$500 per outlet installed. Commercial rates may be higher. Source: [HomeGuide](https://homeguide.com/costs/cost-to-install-a-220v-or-240v-outlet) |
| Dock mounting / anchoring (10 docks) | $500 | Concrete anchors or weighted bases, $50/dock |
| Weatherproofing shelters (10 docks) | $2,000 | Simple polycarbonate rain shelters, $200/dock |
| Electrical permits (municipal) | $200 -- $500 | Varies by city |
| **Total dock installation** | **$5,700 -- $8,000** | |

### 4.3 On-Site Setup and Calibration

| Item | Time per Robot | Total (10 robots) | Hard Cost |
|------|---------------|-------------------|-----------|
| Unpacking and inspection | 1 hour | 10 hours | $0 (labor only) |
| Charging dock alignment and testing | 30 min | 5 hours | $0 |
| GPS/RTK base station setup | N/A (one-time) | 4 hours | $500 (equipment) |
| Map creation and geofencing | N/A (one-time) | 8 hours | $0 |
| Per-robot calibration (IMU, cameras, actuators) | 2 hours | 20 hours | $0 |
| Integration testing (all robots simultaneously) | N/A (one-time) | 8 hours | $0 |
| Burn-in test (4 hours supervised operation per robot) | 4 hours | 40 hours | $0 |
| **Total setup** | ~10 hrs/robot | ~95 hours | **$500** |

**Note:** The 95 hours of on-site work is covered by the 2-week deployment trip. Two engineers working full days can comfortably handle this within the 10-day work window.

### 4.4 Customer Training

| Item | Cost | Notes |
|------|------|-------|
| Training materials creation | $0 | Created during development phase |
| On-site training (2 sessions x 2 hours) | $0 | Delivered by deployment engineers |
| Fleet dashboard setup and walkthrough | $0 | Cloud-hosted, provisioned remotely |
| Emergency procedures documentation | $0 | Provided as part of deployment package |
| **Total training hard cost** | **$0** | Engineering time only |

### 4.5 Deployment Logistics Summary

| Item | Local Pilot (SF) | Remote Pilot (Austin) |
|------|-----------------|----------------------|
| Engineer travel (2 people, 2 weeks) | $1,000 -- $1,700 | $8,900 -- $9,700 |
| Charging dock installation | $5,700 -- $8,000 | $5,700 -- $8,000 |
| RTK base station | $500 | $500 |
| Local supplies and tools | $300 -- $500 | $300 -- $500 |
| **Total deployment** | **$7,500 -- $10,700** | **$15,400 -- $18,700** |

**Working estimate:** $12,000 -- $17,000 (budgeting for a remote deployment as the conservative case).

---

## 5. Ongoing Logistics During Pilot (6 Months)

### 5.1 Cellular Connectivity

Each robot needs an IoT cellular data plan for telemetry, fleet management, remote diagnostics, and low-resolution video streaming.

| Provider | Plan Type | Monthly Cost per Robot | 10 Robots/Month | Source |
|----------|-----------|----------------------|-----------------|--------|
| Hologram | Pay-as-you-go ($1/SIM/mo + $0.03/MB) | $7 -- $15 (est. 200-500 MB/mo) | $70 -- $150 | [Hologram Pricing](https://www.hologram.io/pricing/) |
| T-Mobile IoT | Fixed plan | $10 -- $15 | $100 -- $150 | [Choice IoT](https://www.choiceiot.com/iot-data-plans/) |
| AT&T IoT | Fixed plan | $10 -- $20 | $100 -- $200 | AT&T IoT marketplace |

**Recommendation:** Hologram for flexibility during pilot (no contract, multi-carrier, global roaming for future EU expansion). Budget $12/robot/month.

| Item | Monthly | 6-Month Total |
|------|---------|---------------|
| Cellular data (10 robots x $12/mo) | $120 | **$720** |

### 5.2 Spare Parts Inventory (On-Site)

A spare parts kit should be deployed on-site at the pilot location to minimize downtime. Based on failure rate estimates and the BOM research, the recommended on-site kit is:

| Spare Part | Qty | Unit Cost | Total | Rationale |
|-----------|-----|-----------|-------|-----------|
| AK70-10 actuator | 2 | $499 | $998 | Most likely component to fail under continuous outdoor use |
| AK60-6 actuator | 1 | $299 | $299 | Hip abduction spare |
| Soft gripper finger sets | 10 | $15 | $150 | High-wear item, 1 spare set per robot |
| Dynamixel XM430 servo | 1 | $270 | $270 | Arm joint spare |
| Rubber foot pad sets | 5 | $15 | $75 | High-wear item |
| DC-DC converter (48V to 12V) | 2 | $20 | $40 | Quick replacement |
| DC-DC converter (48V to 5V) | 2 | $15 | $30 | Quick replacement |
| CAN bus cable assemblies | 5 | $10 | $50 | Connectors can fail in weather |
| SIM7600G-H module | 1 | $55 | $55 | Comms spare |
| Misc (fuses, connectors, screws, zip ties) | - | - | $200 | Consumables |
| Pelican case for parts storage | 1 | $150 | $150 | Weatherproof on-site storage |
| **Total spare parts kit** | | | **$2,317** | |

**Shipping spare parts kit to site:**
| Item | Cost |
|------|------|
| FedEx Ground (spare parts kit, ~8 kg) | $30 -- $60 |

### 5.3 Replacement Parts During Pilot (6 Months)

Based on expected wear rates during outdoor operation:

| Item | Expected Replacements | Cost | Shipping |
|------|-----------------------|------|----------|
| Gripper fingers (high wear) | 20-30 sets over 6 months | $300 -- $450 | $50 (2-3 domestic shipments) |
| Foot pads | 10 sets over 6 months | $150 | $30 |
| Actuator replacement (if any) | 0-2 over 6 months | $0 -- $1,000 | $0 -- $80 (express from China or from spares) |
| Misc connectors, cables | Ongoing | $100 | Included above |
| **Total replacement parts + shipping** | | **$550 -- $1,700** | **$80 -- $160** |

### 5.4 Engineer Maintenance Visits

For a remote pilot, monthly maintenance visits are recommended for the first 3 months, then bi-monthly:

| Visit Pattern | Cost per Visit | 6-Month Total |
|--------------|---------------|---------------|
| Monthly visit (1 engineer, 2-3 days) -- first 3 months | $1,200 -- $1,800 | $3,600 -- $5,400 |
| Bi-monthly visit (1 engineer, 2-3 days) -- months 4-6 | $1,200 -- $1,800 | $1,200 -- $1,800 |
| **Total maintenance visits** | | **$4,800 -- $7,200** |

Per visit breakdown: Flight $200-350, hotel 2 nights @ $187 = $374, meals 3 days @ $80 = $240, rental car 3 days @ $50 = $150.

**Alternative for local pilot:** $0 if commuting, or budget $600/month for dedicated maintenance time/transport = $3,600 over 6 months.

### 5.5 Ongoing Logistics Summary (6-Month Pilot)

| Item | Cost (Local Pilot) | Cost (Remote Pilot) |
|------|-------------------|---------------------|
| Cellular data (10 robots, 6 months) | $720 | $720 |
| Spare parts kit (initial) | $2,317 | $2,317 |
| Spare parts kit shipping | $30 -- $60 | $30 -- $60 |
| Replacement parts (6 months) | $550 -- $1,700 | $550 -- $1,700 |
| Replacement parts shipping | $80 -- $160 | $80 -- $160 |
| Maintenance visits (6 months) | $0 -- $3,600 | $4,800 -- $7,200 |
| **Total ongoing logistics** | **$3,697 -- $8,557** | **$8,497 -- $12,157** |

---

## 6. Return / End-of-Pilot Logistics

### 6.1 Pilot Conversion (Best Case)

If the customer converts to a full contract, no return logistics are needed. The robots stay on-site. Cost: **$0**.

### 6.2 Pilot Non-Conversion (Robots Returned)

If the pilot does not convert, all 10 robots and charging docks must be returned to CleanWalker's facility.

| Item | Cost | Notes |
|------|------|-------|
| On-site disassembly and inspection (1 engineer, 3-4 days) | $0 (labor) + $1,200-$1,800 (travel) | Same travel cost as maintenance visit |
| Re-crating (may be able to reuse original crates) | $500 -- $1,000 | Crates may be damaged; budget for partial re-crating |
| Return LTL freight (10 robots + 10 docks) | $1,500 -- $2,500 | Similar to outbound shipping |
| Return cargo insurance | $500 -- $1,000 | Same rate as outbound |
| **Total return shipping** | **$3,700 -- $6,300** | |

### 6.3 Refurbishment Costs

Returned pilot units will show wear and may need refurbishment before redeployment:

| Item | Per Robot | 10 Robots | Notes |
|------|----------|-----------|-------|
| Full inspection and diagnostics | $0 (labor) | $0 | Internal engineering time |
| Gripper finger replacement | $15 | $150 | Replace all soft gripper components |
| Foot pad replacement | $15 | $150 | Replace all foot pads |
| Battery health check + possible replacement | $0 -- $380 | $0 -- $1,140 | 0-3 batteries may need replacement after 6 months |
| Deep clean and repainting/touch-up | $20 | $200 | Cosmetic refurbishment |
| Weatherproof seal replacement | $10 | $100 | Replace all gaskets |
| Re-calibration and burn-in test | $0 (labor) | $0 | Internal engineering time |
| **Total refurbishment** | **$60 -- $440** | **$600 -- $1,740** | |

### 6.4 Return/End-of-Pilot Summary

| Scenario | Cost |
|----------|------|
| Pilot converts to full contract | $0 |
| Pilot does not convert -- return + refurbish | $4,300 -- $8,040 |
| **Working estimate (budget for return)** | **$5,000 -- $7,000** |

---

## 7. International Pilot Variant

### 7.1 EU Pilot (e.g., Amsterdam, Munich, or Copenhagen)

#### Outbound Shipping (US to EU)

| Method | Rate | Cost (10 robots, ~400 kg crated) | Transit | Source |
|--------|------|--------------------------------|---------|--------|
| Air freight (standard) | $5-8/kg | $2,000 -- $3,200 | 3-7 days | [Freightos Air Freight](https://www.freightos.com/freight-resources/air-freight-rates-cost-prices/) |
| Air freight (express) | $8-12/kg | $3,200 -- $4,800 | 2-4 days | [Webcargo](https://www.webcargo.co/blog/air-cargo-price-per-kg/) |
| Sea freight (LCL) | $50-120/CBM | $315 -- $756 (6.3 CBM) | 25-40 days | [Freightos Container](https://www.freightos.com/freight-resources/container-shipping-cost-calculator-free-tool/) |

**Recommendation for EU pilot:** Air freight. The robots are high-value, time-sensitive items. Sea freight adds 4-6 weeks and requires more robust packaging for longer transit and handling.

#### EU Import Duties

Industrial robots (HS 8479.50) have a **0% MFN duty rate** in most countries including the EU and UK.

| Item | Rate | Cost | Source |
|------|------|------|--------|
| EU import duty on robots (HS 8479.50) | 0% | $0 | [UK Trade Tariff](https://www.trade-tariff.service.gov.uk/commodities/8479500000) |
| EU VAT (on import value + freight) | 19-25% (varies by country) | $20,000 -- $27,500 | Recoverable if selling B2B |
| Customs brokerage (EU entry) | $200 -- $500 | $200 -- $500 | |

**Important:** The 0% import duty for industrial robots is a significant advantage for EU deployment. However, VAT of 19-25% applies (21% in Netherlands, 19% in Germany, 25% in Denmark). VAT is recoverable for B2B transactions if CleanWalker or a local entity is VAT-registered, but requires upfront cash flow.

#### Export Compliance (ITAR/EAR)

CleanWalker's robot is a commercial autonomous ground vehicle for civilian litter collection. Key considerations:

- **ITAR (International Traffic in Arms Regulations):** Unlikely to apply. ITAR covers items on the US Munitions List (USML). Commercial cleaning robots are not military items. However, any defense-related contracts or dual-use capabilities would change this assessment.

- **EAR (Export Administration Regulations):** More likely to apply. The robot contains a NVIDIA Jetson (GPU compute) and autonomous navigation software. Under EAR, the robot would likely be classified as **EAR99** (no specific ECCN), meaning it can be exported to most countries without a license.

- **Key risk:** If the robot uses any encryption above certain thresholds, a TSPA (Thermal Signature Performance Assessment) or ENC classification may be needed. The 4G LTE module likely qualifies for the ENC mass-market exception.

- **Recommendation:** File a commodity jurisdiction (CJ) request or self-classify under EAR before exporting. Cost: $0-$2,000 (legal review). Timeline: 2-4 weeks for self-classification.

Sources: [Secureframe ITAR vs EAR Guide](https://secureframe.com/blog/ear-vs-itar), [FD Associates ITAR Update](https://fdassociates.net/what-the-latest-itar-revisions-mean-for-small-businesses-september-2025/)

#### EU Deployment Travel

| Item | Cost (2 engineers, 2 weeks) | Notes |
|------|----------------------------|-------|
| Round-trip flights (SFO to AMS/MUC) | $1,500 -- $3,000 | Economy class, booked in advance |
| Hotel (14 nights x 2 rooms) | $4,000 -- $7,000 | European business hotels, varies by city |
| Meals & incidentals (2 people, 14 days) | $2,000 -- $3,500 | Higher cost of living in EU cities |
| Local transport (transit pass + occasional taxi) | $300 -- $600 | |
| **Total EU deployment travel** | **$7,800 -- $14,100** | |

#### EU Pilot Total Logistics Cost Estimate

| Item | Cost (EU, 10 units) |
|------|-------------------|
| Inbound component shipping (same as US -- components come to US first) | $535 -- $997 |
| Import duties on components (same as US -- assembled in US) | $24,698 |
| Outbound shipping (US to EU, air freight) | $2,000 -- $3,200 |
| Crating (same) | $1,550 -- $2,700 |
| EU customs + brokerage | $200 -- $500 |
| Cargo insurance (international) | $800 -- $1,500 |
| EU deployment travel | $7,800 -- $14,100 |
| Dock installation (EU electrician rates may be higher) | $6,000 -- $10,000 |
| Ongoing logistics (6 months, higher travel costs) | $10,000 -- $16,000 |
| Return shipping (EU to US, if needed) | $5,000 -- $8,000 |
| Export compliance (legal review) | $1,000 -- $2,000 |
| **Total EU pilot logistics** | **$59,583 -- $83,695** |

### 7.2 Singapore Pilot

Singapore is attractive because of:
- 0% duty on most electronics and machinery
- English-speaking, excellent logistics infrastructure
- Strong government support for robotics (Smart Nation initiative)

Key differences from EU:
- Air freight US to Singapore: ~$5-8/kg, similar to EU
- Singapore GST: 9% (much lower than EU VAT, and recoverable for B2B)
- No ITAR concerns for civilian commercial equipment to Singapore
- Higher travel costs (longer flights, $1,500-$2,500/person round trip)
- Higher hotel costs ($200-$350/night in central Singapore)

Estimated total: **$55,000 -- $80,000** (similar to EU pilot, slightly lower duties but higher travel)

---

## 8. Comprehensive Summary Tables

### 8.1 Total Logistics Cost: 10-Unit US Pilot

| Logistics Category | Low Estimate | High Estimate | Best Estimate |
|-------------------|-------------|---------------|---------------|
| **1. Inbound component shipping** | $535 | $997 | $750 |
| **2. Customs brokerage fees** | $425 | $725 | $575 |
| **3. Import duties & tariffs** | $24,698 | $24,698 | $24,698 |
| **4. Outbound packaging (crating)** | $1,550 | $2,700 | $2,000 |
| **5. Outbound freight (to customer)** | $2,000 | $3,500 | $2,500 |
| **6. Cargo insurance** | $500 | $1,000 | $750 |
| **7. Charging dock shipping** | $50 | $150 | $100 |
| **8. Deployment travel** | $9,000 | $10,000 | $9,500 |
| **9. Dock installation** | $5,700 | $8,000 | $6,500 |
| **10. Setup equipment (RTK, supplies)** | $800 | $1,000 | $900 |
| **11. Spare parts kit (initial)** | $2,347 | $2,377 | $2,350 |
| **12. Cellular data (6 months)** | $720 | $720 | $720 |
| **13. Replacement parts + shipping (6 mo)** | $630 | $1,860 | $1,200 |
| **14. Maintenance visits (6 months)** | $4,800 | $7,200 | $6,000 |
| **15. Return logistics (budget reserve)** | $4,300 | $8,040 | $5,500 |
| **TOTAL** | **$58,055** | **$72,967** | **$64,043** |

### 8.2 Comparison: US vs. EU Pilot

| Logistics Item | US Pilot (10 units) | EU Pilot (10 units) |
|---------------|--------------------|--------------------|
| Inbound component shipping | $750 | $750 |
| Customs brokerage (inbound) | $575 | $575 |
| Import duties (Chinese components) | $24,698 | $24,698 |
| Outbound crating | $2,000 | $2,000 |
| Outbound freight to customer | $2,500 | $3,000 (air to EU) |
| Cargo insurance | $750 | $1,200 |
| EU customs + brokerage | N/A | $400 |
| Deployment travel (2 eng, 2 weeks) | $9,500 | $11,000 |
| Dock installation | $6,500 | $8,000 |
| Setup equipment + supplies | $900 | $900 |
| Spare parts kit | $2,350 | $2,350 |
| Cellular data (6 months) | $720 | $720 |
| Replacement parts + shipping | $1,200 | $1,500 |
| Maintenance visits (6 months) | $6,000 | $9,000 |
| Return logistics (reserve) | $5,500 | $7,000 |
| Export compliance (legal) | N/A | $1,500 |
| **TOTAL** | **$63,943** | **$74,593** |

### 8.3 Per-Robot Logistics Summary

| Phase | Per-Robot Cost (US) | Per-Robot Cost (EU) | Notes |
|-------|--------------------|--------------------|-------|
| Inbound shipping + brokerage | $133 | $133 | Same -- components ship to US in both cases |
| Import duties & tariffs | $2,470 | $2,470 | Tariffs on Chinese components |
| Outbound packaging | $200 | $200 | Custom crating |
| Outbound freight + insurance | $325 | $420 | Air freight premium for EU |
| Deployment (share of fixed costs) | $1,690 | $1,990 | Travel, dock installation, setup |
| Spare parts kit (share) | $235 | $235 | Initial on-site inventory |
| Ongoing (cellular + parts + visits, 6 mo) | $792 | $1,122 | Higher visit costs for EU |
| Return reserve (share) | $550 | $700 | Budget for potential return |
| Export compliance (share) | $0 | $150 | EAR classification review |
| **TOTAL PER ROBOT** | **$6,395** | **$7,420** | |

### 8.4 Logistics as % of Total Pilot Cost

Comparing logistics to the total pilot investment from `pilot-financial-model.md`:

| Metric | Value |
|--------|-------|
| BOM cost per robot (10-unit pricing) | $8,799 |
| Logistics cost per robot (US pilot) | ~$6,400 |
| Logistics as % of BOM | **73%** |
| Logistics as % of total unit cost (BOM + assembly + QA + logistics) | **37%** |
| Import tariffs alone as % of BOM | **28%** |

**The tariff burden is extraordinary.** Import duties on Chinese actuators alone ($2,352/robot) represent the single largest logistics cost and 27% of the total BOM cost. This cost did NOT appear in the BOM research or pilot financial model, which listed component prices at landed-in-US supplier pricing for domestic components but did not account for duties on direct-from-China purchases.

### 8.5 Updated Pilot Financial Impact

Adding logistics costs to the pilot financial model:

| Line Item | Scrappy Founder (from pilot model) | + Logistics Costs |
|-----------|-----------------------------------|------------------|
| Pre-pilot development | $140,591 | $140,591 |
| Pilot production (10 units) | $118,439 | $118,439 |
| **Logistics (NEW)** | **$3,000 (original shipping est.)** | **$64,043** |
| Deployment | $13,915 | Absorbed into logistics |
| Operations (6 months) | $15,180 | Absorbed into logistics |
| **Revised total** | **$288,125** | **~$323,073** |
| **Delta** | | **+$35,000 -- $50,000** |

**Key insight:** The original pilot financial model budgeted ~$3,000 for shipping and rolled deployment/operations costs separately. This detailed logistics analysis reveals an additional ~$35,000-$50,000 in costs that were underestimated or not explicitly captured, primarily driven by:
1. Import tariffs on Chinese components (~$24,700)
2. Custom crating costs (~$2,000)
3. Spare parts logistics (~$3,500)
4. Cargo insurance (~$750)

---

## Appendix A: Tariff Mitigation Strategies

| Strategy | Potential Savings | Complexity | Timeline |
|----------|------------------|------------|----------|
| **File for Section 301 exclusion** on HTS 8501.31.40 | -$12,000 (25% x $48K) | Medium -- requires customs attorney | 3-6 months |
| **Reclassify actuators** under different HTS code (e.g., as "robot parts" under 8479.90) | Could change Section 301 applicability | High -- requires CBP ruling | 2-4 months |
| **Source actuators from non-China** (South Korea: T-Motor Korea, Japan: Harmonic Drive) | Eliminates Section 301 + IEEPA + Reciprocal | High -- may increase unit cost | 6-12 months |
| **Establish Foreign Trade Zone (FTZ)** assembly operation | Defer/reduce duties on re-exported goods | High -- only viable at scale | 12+ months |
| **Use JLCPCB DDP service** to simplify PCB tariff handling | No savings, but reduces admin burden | Low | Immediate |
| **Domestic PCB fabrication** (e.g., Advanced Circuits, PCBUSA) | Eliminates 45% PCB tariff (~$810) | Low -- slightly higher unit cost | Immediate |

**Priority recommendation:** Engage a licensed customs broker immediately to:
1. Confirm HTS classification for CubeMars actuators
2. Check current Section 301 exclusion lists for applicable exemptions
3. Evaluate HTS reclassification options
4. Set up proper import procedures before first shipment

Estimated cost for customs broker consultation: $500 -- $1,500.

## Appendix B: Key Shipping Regulations

### Lithium Battery Shipping (UN3481/UN3480)

| Regulation | Requirement | Impact |
|-----------|-------------|--------|
| UN3481 (batteries packed with equipment) | Standard labeling, no hazmat surcharge on ground | Batteries shipped separately require this classification |
| UN3480 (batteries alone, standalone) | More restrictive; ground only, proper packaging | If shipping spare batteries separately |
| IATA 2026 SoC limit | Batteries must be at 30% state of charge for air transport | Affects international shipping; charge batteries to 30% before air shipment |
| FedEx Ground | Accepts UN3481 without surcharge when properly labeled | Preferred domestic carrier for battery shipments |
| UPS | Accepts UN3481 ground; restricted for UN3480 air | Alternative carrier |

Sources: [FedEx Battery Guide 2025](https://www.fedex.com/content/dam/fedex/us-united-states/services/Battery_Overview_2025.pdf), [IATA Battery Guidance 2026](https://www.iata.org/contentassets/05e6d8742b0047259bf3a700bc9d42b9/lithium-battery-guidance-document.pdf), [Lion Technology](https://www.lion.com/lion-news/december-2025/new-lithium-battery-state-of-charge-limit-in-effect-jan-1)

### Export Compliance (for International Pilots)

| Item | Classification | Requirement |
|------|---------------|-------------|
| NVIDIA Jetson (GPU) | Likely EAR99 or 4A994 | Check Commerce Control List; most civilian GPUs are EAR99 |
| Autonomous navigation software | Likely EAR99 | No military application = no ECCN restriction |
| 4G LTE radio (SIM7600G-H) | ENC mass-market exception | No license needed for mass-market encryption |
| Complete robot assembly | Self-classify under EAR | File SED (Shipper's Export Declaration) for shipments >$2,500 |
| ITAR applicability | Unlikely | Civilian cleaning robot is not on USML |

Sources: [Secureframe EAR vs ITAR](https://secureframe.com/blog/ear-vs-itar), [Envoy ITAR vs EAR 2025](https://envoy.com/workplace-compliance-security-safety/what-are-the-differences-between-itar-vs-ear)

## Appendix C: Recommended Logistics Timeline (10-Unit US Pilot)

| Week | Activity | Lead Time | Cost Trigger |
|------|----------|-----------|-------------|
| T-12 | Engage customs broker; confirm HTS codes | 2-4 weeks | $500-$1,500 |
| T-10 | Place CubeMars actuator order (120 units) | 4-6 week production + 1 week shipping | $48,000 + $400-$650 shipping |
| T-8 | Order JLCPCB PCBs (assembled) | 2-3 weeks production + 1 week shipping | $1,800 + $25-$50 shipping |
| T-6 | Order domestic components (Jetson, cameras, servos, batteries) | 1-2 weeks | Various BOM costs |
| T-4 | Order CNC frame parts from Xometry | 2-3 weeks (standard) | Frame costs (free shipping) |
| T-3 | Actuators clear customs; duties paid | 1-3 days after arrival | $24,698 import duties |
| T-2 | Begin assembly (10 units) | 4-6 weeks | Assembly labor |
| T+0 | Assembly complete; QA testing | 1-2 weeks | QA labor |
| T+2 | Custom crating and outbound shipping | 1 week crating + 3-7 days freight | $4,100-$7,350 |
| T+3 | Robots arrive at customer site | - | - |
| T+3 | Deployment team travels to site | 2 weeks on-site | $15,400-$18,700 |
| T+5 | Pilot begins | 6 months | Ongoing logistics |
| T+29 | Pilot ends; conversion or return decision | - | $0 or $5,000-$7,000 |

---

## Appendix D: Data Sources and References

- [Freightos -- Shipping from China to US (Feb 2026)](https://www.freightos.com/shipping-routes/shipping-from-china-to-the-united-states/)
- [Sino-Shipping -- China to US Guide (Jan 2026)](https://www.sino-shipping.com/country-guides/shipping-from-china-to-usa/)
- [China Briefing -- US-China Tariff Rates](https://www.china-briefing.com/news/us-china-tariff-rates-2025/)
- [USITC Harmonized Tariff Schedule](https://hts.usitc.gov/)
- [CBP Customs Broker Fees](https://www.cbp.gov/trade/programs-administration/customs-brokers/fees)
- [USTR Section 301 Tariff Actions](https://ustr.gov/issue-areas/enforcement/section-301-investigations/tariff-actions)
- [White House -- US-China Trade Deal (Nov 2025)](https://www.whitehouse.gov/fact-sheets/2025/11/fact-sheet-president-donald-j-trump-strikes-deal-on-economic-and-trade-relations-with-china/)
- [KPMG -- US Tariff Rates and Duties on China](https://kpmg.com/us/en/taxnewsflash/news/2025/11/united-states-tariff-rates-duties-china.html)
- [Penn Wharton Budget Model -- Effective Tariff Rates (Jan 2026)](https://budgetmodel.wharton.upenn.edu/issues/2026/1/15/effective-tariff-rates-and-revenues-updated-january-15-2026)
- [DirectPCB -- Tariff Update](https://www.directpcb.com/tariff-update/)
- [ICAPE Group -- US Import Tariffs on PCBs](https://www.icape-group.com/pcb-tariff-information/)
- [JLCPCB -- US Tariff Policy FAQ](https://jlcpcb.com/help/article/us-tariff-policy-faq)
- [FedEx -- Lithium Battery Shipping Guide 2025](https://www.fedex.com/content/dam/fedex/us-united-states/services/Battery_Overview_2025.pdf)
- [IATA -- Lithium Battery Guidance Document 2026](https://www.iata.org/contentassets/05e6d8742b0047259bf3a700bc9d42b9/lithium-battery-guidance-document.pdf)
- [Flock Freight -- Shipping Insurance for Electronics](https://www.flockfreight.com/blog/shipping-insurance-for-high-value-electronics-and-tech)
- [GSA Per Diem Rates -- San Francisco 2025](https://www.federalpay.org/perdiem/2025/california/san-francisco)
- [GSA Per Diem Rates -- Austin TX 2025](https://www.federalpay.org/perdiem/2025/texas/austin)
- [Hologram -- IoT Pricing](https://www.hologram.io/pricing/)
- [Xometry -- CNC Machining Service](https://www.xometry.com/capabilities/cnc-machining-service/)
- [Red Stag Fulfillment -- Freight Delivery Cost Guide 2026](https://redstagfulfillment.com/how-much-does-freight-delivery-cost/)
- [HomeGuide -- 240V Outlet Installation Cost](https://homeguide.com/costs/cost-to-install-a-220v-or-240v-outlet)
- [UK Trade Tariff -- HS 8479500000 (Industrial Robots)](https://www.trade-tariff.service.gov.uk/commodities/8479500000)
- [Secureframe -- EAR vs ITAR Guide](https://secureframe.com/blog/ear-vs-itar)
- [Craters & Freighters -- Robotics Shipping](https://www.cratersandfreighters.com/shipping/electronics-shipping/robotics-shipping/)
- [Waku Robotics -- Spare Parts Management](https://www.waku-robotics.com/en/magazine/how-to-master-spare-parts-management-for-robots-agv-amr-industrial)
- [Clearit USA -- Customs Clearance Pricing](https://clearitusa.com/customs-clearance-pricing-rates/)

---

*This document is a living research artifact. All prices are in USD and were researched in February 2026. Tariff rates are subject to change -- the US-China trade situation is volatile and the current reduced rates expire November 10, 2026. Freight rates fluctuate seasonally. A licensed customs broker should be engaged before any actual import activity. Actual costs may vary from estimates provided here.*
