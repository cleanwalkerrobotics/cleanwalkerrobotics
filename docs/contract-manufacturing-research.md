# Contract Manufacturing Research: CleanWalker Robot Assembly
**Date:** 2026-02-09
**Version:** 1.0
**Purpose:** Identify US and EU contract manufacturers for electromechanical robot assembly with realistic per-unit costs, NRE, and volume pricing

---

## Executive Summary

For a complex electromechanical robot like CleanWalker (~30kg, 12 actuators, gripper arm, cameras, compute, battery), contract manufacturing (CM) costs vary dramatically by volume:

**US Contract Manufacturing - Per Unit Assembly Cost:**
- 10 units: $4,500-$6,500/unit (50-65 labor hours @ $70-100/hr burdened)
- 50 units: $2,800-$4,000/unit (35-45 hrs @ $70-90/hr)
- 100 units: $2,000-$3,000/unit (28-38 hrs @ $65-80/hr)
- 500 units: $1,200-$2,000/unit (18-28 hrs @ $60-75/hr)

**EU Contract Manufacturing - Per Unit Assembly Cost:**
- 10 units: €3,800-€5,500/unit (45-60 hrs @ €70-95/hr burdened)
- 50 units: €2,400-€3,400/unit (32-42 hrs @ €65-85/hr)
- 100 units: €1,700-€2,600/unit (25-35 hrs @ €60-75/hr)
- 500 units: €1,000-€1,700/unit (16-25 hrs @ €55-70/hr)

**NRE (Non-Recurring Engineering) Setup:**
- US: $75,000-$180,000
- EU: €65,000-€150,000

**Key Finding:** US labor rates are 15-25% higher than EU (Germany/Netherlands), but US CMs offer faster iteration cycles and better robotics specialization. EU offers cost advantages at volume but 20-35% longer lead times.

---

## Table of Contents

1. [US Contract Manufacturers](#1-us-contract-manufacturers)
2. [EU Contract Manufacturers](#2-eu-contract-manufacturers)
3. [Cost Model by Volume](#3-cost-model-by-volume)
4. [NRE Breakdown](#4-nre-breakdown)
5. [Quality & Testing Costs](#5-quality--testing-costs)
6. [Learning Curve Analysis](#6-learning-curve-analysis)
7. [Recommendations](#7-recommendations)
8. [Sources](#8-sources)

---

## 1. US Contract Manufacturers

### 1.1 Qualified US Robotics CMs

| Company | Location | Specialization | Min Volume | Lead Time | Notes |
|---------|----------|---------------|-----------|-----------|-------|
| **Mack Molding** | Vermont | Full robotics assembly, actuators, LiDAR integration | 5+ units | 6-10 weeks | Strong in autonomous systems; handles volumes 5-5M units |
| **PEKO Precision** | Rochester, NY | Electromechanical assembly, 375k sq ft facility | 1+ units | 4-8 weeks | Vertically integrated; flexible on low volumes |
| **Fathom Manufacturing** | Multiple US locations | Robotics assembly, low-volume/high-mix | 1+ units | 4-8 weeks | Strong DFM support; good for prototypes |
| **Simplexity Product Development** | San Diego, CA | Premium engineering + NPI | 1+ units | 6-12 weeks | High-cost but includes engineering; Unitree Go2 teardown expertise |
| **Jabil** | Multiple US sites | High-volume EMS, robotics automation | 100+ units | 8-16 weeks | Cost-effective at scale; less flexible at low volumes |
| **Flex** | Multiple US sites | Large-scale robotics, medical devices | 50+ units | 8-16 weeks | Global supply chain strength |
| **Benchmark Electronics** | Multiple US sites | Mid-volume electronics + mechanical | 25+ units | 6-12 weeks | Good middle ground for 50-500 unit range |

### 1.2 US Labor Rates (2026)

**Base Manufacturing Wages:**
- Electromechanical technician: $28-36/hr (base wage)
- Skilled robotics technician: $34-42/hr (base wage)
- Average manufacturing wage (US): $29.51/hr (Dec 2025 BLS data)

**Burdened Labor Rates (What CMs Charge):**
- Small CM (PEKO, Mack): $70-100/hr (includes 25-35% burden + facility overhead + profit)
- Mid-tier CM (Benchmark): $60-85/hr
- Large CM (Jabil, Flex): $55-75/hr (scale economies)

**Burden Calculation:**
- Base wage: $30-36/hr
- Employer taxes & benefits (30-40%): +$9-14/hr
- Facility overhead (40-60%): +$16-24/hr
- Profit margin (15-25%): +$12-20/hr
- **Total burdened rate: $67-94/hr**

### 1.3 US Assembly Time Estimates by Volume

**CleanWalker Complexity Factors:**
- 12 actuators with individual CAN bus configuration
- Complex wiring harness (48V power + CAN + sensors)
- Gripper arm with servo integration
- Jetson Orin Nano compute + camera calibration
- IP65 weatherproofing (gaskets, conformal coating)
- Functional testing (locomotion, gripper, sensors)

**Labor Hours per Unit:**

| Volume | Units | Labor Hrs/Unit | Reduction from Previous | Notes |
|--------|-------|----------------|------------------------|-------|
| First unit (prototype) | 1 | 75-95 hrs | Baseline | Heavy documentation, debugging, process development |
| Low volume | 10 | 55-70 hrs | -25% | Assembly procedures documented; still some manual work |
| Small production | 50 | 40-50 hrs | -30% | Fixtures in place; workers trained; some specialization |
| Mid production | 100 | 32-42 hrs | -20% | Dedicated assembly line; work cells; quality procedures |
| High production | 500 | 22-32 hrs | -30% | Optimized fixtures; flow production; strong learning curve effect |

**Learning Curve:** 80% learning curve is standard for electromechanical assembly (every doubling of cumulative volume reduces labor by 20%). For 10 to 100 units, expect 35-45% labor reduction.

---

## 2. EU Contract Manufacturers

### 2.1 Qualified EU Robotics CMs

| Company | Location | Specialization | Min Volume | Lead Time | Notes |
|---------|----------|---------------|-----------|-----------|-------|
| **OMRON Manufacturing Netherlands** | Den Bosch, NL | High-mix low-volume production, HMI, robots | 10+ units | 8-12 weeks | Strong in industrial automation; EU supply chain |
| **Quimesis** | Belgium | Robotics system assembly, automation | 5+ units | 6-10 weeks | Full-service from design to assembly; 10+ years experience |
| **Zign Group (Contract Mfg)** | Netherlands | Electronics + mechanical assembly | 10+ units | 8-12 weeks | Experienced in complex electromechanical products |
| **Dekimo** | Belgium/Netherlands | Electronics manufacturing, robotics | 5+ units | 6-10 weeks | Strong in embedded systems + mechanical integration |
| **Generic EU EMS Providers** | Germany, Poland | Large-scale EMS with robotics capability | 50+ units | 10-16 weeks | Cost-competitive at volume; longer lead times |

**Note:** EU robotics CM market is less mature than US for quadruped robots specifically. Most EU CMs focus on industrial automation (robotic arms, AGVs) rather than legged robots. US has more specialized expertise.

### 2.2 EU Labor Rates (2026)

**Base Manufacturing Wages:**
- Germany manufacturing worker: €17/hr (gross hourly wage)
- Netherlands production worker: €13-14/hr (excluding holiday pay)
- EU average labor cost: €31.80/hr (2023 Eurostat data, includes all burdens)
- Netherlands minimum wage: €14.71/hr (2026)

**Burdened Labor Rates (What EU CMs Charge):**
- Small EU CM (Quimesis, Dekimo): €70-95/hr
- Mid-tier EU CM: €60-80/hr
- Large EU EMS (Poland/Czech): €50-70/hr

**Labor Cost Comparison (US vs EU):**
- US burdened rate: $70-100/hr = €65-93/hr (at 1.08 USD/EUR)
- EU burdened rate: €60-95/hr
- **Net difference: US is 5-15% more expensive at similar CM tiers**

However, EU lead times are typically 20-35% longer for prototype/low-volume work due to:
- Less specialized robotics CM capacity
- More conservative approach to NPI
- Language/cultural barriers for US-based companies

### 2.3 EU Assembly Time Estimates

EU labor hours are comparable to US for the same complexity level:

| Volume | Labor Hrs/Unit (EU) | Burdened Rate | Cost/Unit |
|--------|---------------------|---------------|-----------|
| 10 units | 50-65 hrs | €70-90/hr | €3,500-€5,850 |
| 50 units | 35-45 hrs | €65-85/hr | €2,275-€3,825 |
| 100 units | 28-38 hrs | €60-75/hr | €1,680-€2,850 |
| 500 units | 20-30 hrs | €55-70/hr | €1,100-€2,100 |

**EU Advantages:**
- Lower labor costs at volume (500+ units)
- Access to EU market without import duties/tariffs
- Strong quality culture (German engineering standards)

**EU Disadvantages:**
- Longer lead times for iteration (8-12 weeks vs 4-8 weeks US)
- Less expertise in quadruped robotics specifically
- Higher communication overhead for US-based team

---

## 3. Cost Model by Volume

### 3.1 US Contract Manufacturing - Full Cost Breakdown

#### 10 Units (First Production Run)

| Cost Element | Per Unit | 10 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | $8,799 | $87,990 | Per existing BOM research at 10-unit pricing |
| Assembly labor (60 hrs @ $75/hr) | $4,500 | $45,000 | Mid-tier US CM rate |
| Quality/testing (8 hrs @ $75/hr) | $600 | $6,000 | Functional test per unit |
| Fixtures/tooling amortization | $800 | $8,000 | NRE amortized over first 10 units |
| **Total per unit** | **$14,699** | **$146,990** | |

#### 50 Units

| Cost Element | Per Unit | 50 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | $7,800 | $390,000 | Volume pricing improves 10-12% |
| Assembly labor (42 hrs @ $70/hr) | $2,940 | $147,000 | Learning curve effect + better rate |
| Quality/testing (6 hrs @ $70/hr) | $420 | $21,000 | Faster testing with procedures |
| Tooling amortization | $200 | $10,000 | Spread over larger volume |
| **Total per unit** | **$11,360** | **$568,000** | |

#### 100 Units

| Cost Element | Per Unit | 100 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | $7,262 | $726,200 | Per existing BOM research |
| Assembly labor (35 hrs @ $68/hr) | $2,380 | $238,000 | Strong learning curve effect |
| Quality/testing (5 hrs @ $68/hr) | $340 | $34,000 | Standardized test procedures |
| Tooling amortization | $100 | $10,000 | Minimal incremental tooling |
| **Total per unit** | **$10,082** | **$1,008,200** | |

#### 500 Units

| Cost Element | Per Unit | 500 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | $6,100 | $3,050,000 | Approaching volume pricing |
| Assembly labor (24 hrs @ $65/hr) | $1,560 | $780,000 | Optimized assembly line |
| Quality/testing (4 hrs @ $65/hr) | $260 | $130,000 | Automated testing where possible |
| Tooling amortization | $40 | $20,000 | Full amortization |
| **Total per unit** | **$7,960** | **$3,980,000** | |

### 3.2 EU Contract Manufacturing - Full Cost Breakdown

#### 10 Units

| Cost Element | Per Unit | 10 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | €8,150 | €81,500 | Similar to US pricing (global component market) |
| Assembly labor (55 hrs @ €75/hr) | €4,125 | €41,250 | Mid-tier EU CM rate |
| Quality/testing (8 hrs @ €75/hr) | €600 | €6,000 | Functional test per unit |
| Fixtures/tooling amortization | €750 | €7,500 | NRE amortized over first 10 units |
| **Total per unit** | **€13,625** | **€136,250** | |

#### 50 Units

| Cost Element | Per Unit | 50 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | €7,220 | €361,000 | Volume pricing |
| Assembly labor (38 hrs @ €70/hr) | €2,660 | €133,000 | Learning curve + better rate |
| Quality/testing (6 hrs @ €70/hr) | €420 | €21,000 | Faster testing |
| Tooling amortization | €200 | €10,000 | |
| **Total per unit** | **€10,500** | **€525,000** | |

#### 100 Units

| Cost Element | Per Unit | 100 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | €6,725 | €672,500 | |
| Assembly labor (32 hrs @ €68/hr) | €2,176 | €217,600 | Strong learning curve |
| Quality/testing (5 hrs @ €68/hr) | €340 | €34,000 | |
| Tooling amortization | €100 | €10,000 | |
| **Total per unit** | **€9,341** | **€934,100** | |

#### 500 Units

| Cost Element | Per Unit | 500 Units Total | Notes |
|--------------|----------|---------------|-------|
| BOM (components) | €5,650 | €2,825,000 | |
| Assembly labor (22 hrs @ €62/hr) | €1,364 | €682,000 | Optimized line |
| Quality/testing (4 hrs @ €62/hr) | €248 | €124,000 | |
| Tooling amortization | €40 | €20,000 | |
| **Total per unit** | **€7,302** | **€3,651,000** | |

### 3.3 Direct Comparison: US vs EU

| Volume | US Cost/Unit | EU Cost/Unit (EUR) | EU Cost/Unit (USD @ 1.08) | US Premium |
|--------|--------------|-------------------|--------------------------|-----------|
| 10 | $14,699 | €13,625 | $14,715 | -0.1% (comparable) |
| 50 | $11,360 | €10,500 | $11,340 | +0.2% (comparable) |
| 100 | $10,082 | €9,341 | $10,088 | -0.1% (comparable) |
| 500 | $7,960 | €7,302 | $7,886 | +0.9% (comparable) |

**Key Finding:** At current exchange rates (1.08 USD/EUR), US and EU contract manufacturing costs are within 1-2% of each other when comparing similar-tier CMs. The real difference is:
- **US advantage:** Faster iteration (4-8 weeks vs 8-12 weeks), more robotics specialization
- **EU advantage:** Better access to EU market, potential tariff avoidance, strong quality culture

---

## 4. NRE Breakdown

NRE (Non-Recurring Engineering) covers one-time costs to set up production:

### 4.1 US NRE Estimate

| NRE Category | Low | Mid | High | Notes |
|--------------|-----|-----|------|-------|
| **Assembly fixtures & tooling** | $15,000 | $30,000 | $60,000 | Custom fixtures for actuator mounting, wiring harness assembly, gripper integration; higher end includes automated test fixtures |
| **Test equipment** | $8,000 | $18,000 | $35,000 | CAN bus test harness, automated locomotion test rig, gripper force sensor, power monitoring |
| **Documentation & procedures** | $5,000 | $12,000 | $20,000 | Assembly work instructions, quality procedures, training materials, BOM management setup |
| **Engineering support (DFM)** | $12,000 | $25,000 | $45,000 | Design for manufacturing review, assembly optimization, supplier qualification |
| **First article inspection (FAI)** | $3,000 | $8,000 | $15,000 | Dimensional inspection, functional verification, quality documentation |
| **Process validation** | $2,000 | $5,000 | $10,000 | Pilot run, yield analysis, process refinement |
| **Project management** | $5,000 | $10,000 | $20,000 | NPI coordination, supplier management, timeline management |
| **Total US NRE** | **$50,000** | **$108,000** | **$205,000** | Typical range: $75K-$180K for complex electromechanical robot |

### 4.2 EU NRE Estimate

| NRE Category | Low (EUR) | Mid (EUR) | High (EUR) | Notes |
|--------------|-----------|-----------|-----------|-------|
| **Assembly fixtures & tooling** | €12,000 | €25,000 | €50,000 | Similar to US but slightly lower labor costs |
| **Test equipment** | €7,000 | €15,000 | €30,000 | |
| **Documentation & procedures** | €4,000 | €10,000 | €18,000 | May require translation to local language |
| **Engineering support (DFM)** | €10,000 | €22,000 | €40,000 | Strong engineering culture, thorough reviews |
| **First article inspection** | €3,000 | €7,000 | €13,000 | |
| **Process validation** | €2,000 | €5,000 | €10,000 | |
| **Project management** | €4,000 | €8,000 | €15,000 | |
| **Total EU NRE** | **€42,000** | **€92,000** | **€176,000** | Typical range: €65K-€150K |

**USD Equivalent (@ 1.08):** $45,360 - $190,080 (typical: $70K-$162K)

### 4.3 NRE Payment Structure

**Typical CM Payment Terms:**
- 50% upfront at contract signing
- 25% at first article approval
- 25% at first production shipment

**Amortization Options:**
- **Pay upfront:** Lower per-unit cost, but high cash requirement
- **Amortize into unit price:** CM adds $500-$2,000/unit for first 50-100 units
- **Hybrid:** Pay 50% upfront, amortize 50% over first production run

**Recommendation for CleanWalker:** Negotiate 30% upfront, 70% amortized over first 50 units. This keeps cash requirement manageable ($23K-$54K upfront) while avoiding excessive per-unit premiums.

---

## 5. Quality & Testing Costs

### 5.1 Quality Cost Elements

| Quality Activity | Per Unit Cost | Notes |
|-----------------|---------------|-------|
| **Incoming inspection** | $50-120 | BOM components verified before assembly |
| **In-process inspection** | $100-200 | Dimensional checks, electrical continuity, torque verification |
| **Functional testing** | $300-600 | Full locomotion test, sensor verification, gripper test, runtime test |
| **Environmental spot-check** | $50-100 | Random IP65 verification, thermal monitoring |
| **Final QA documentation** | $50-100 | Test reports, serial number tracking, traceability |
| **Total per unit** | **$550-$1,120** | Typically 5-10% of total unit cost |

### 5.2 Testing Infrastructure (One-Time)

| Test Equipment | Cost | Included in NRE? |
|----------------|------|------------------|
| Automated CAN bus test harness | $5,000-$12,000 | Yes |
| Locomotion test area (flat surface + ramps) | $2,000-$5,000 | Yes |
| Gripper test rig (force sensors, object library) | $3,000-$8,000 | Yes |
| Power monitoring system | $1,500-$4,000 | Yes |
| Environmental test chamber (rental/access) | $500-$2,000/month | No (recurring) |
| **Total** | **$12,000-$31,000** | Included in NRE estimate above |

### 5.3 Quality Standards & Certifications

**CM Quality Certifications (Typical):**
- ISO 9001:2015 (Quality Management) - Standard for all CMs
- ISO 13485 (Medical Devices) - Mack, PEKO, Flex offer this
- AS9100 (Aerospace) - Indicates high-quality processes
- IPC-A-610 (Electronics Assembly) - Standard for electronics work

**Quality Cost Impact:**
- ISO 9001 CM: Baseline pricing
- ISO 13485 CM: +10-15% premium (but higher quality)
- No certification: -15-20% cost (but higher risk)

**Recommendation:** Use ISO 9001 certified CM minimum. ISO 13485 preferred if available without major premium.

---

## 6. Learning Curve Analysis

### 6.1 Learning Curve Theory

**80% Learning Curve:** Standard for electromechanical assembly
- Definition: Every doubling of cumulative production reduces labor hours by 20%
- Example: If unit 1 takes 100 hours, unit 2 takes 80 hours average (160 total / 2), unit 4 takes 64 hours average, etc.

**Historical Data:**
- Aircraft manufacturing: 80% learning curve (Theodore Wright, 1936)
- Complex electromechanical products: 75-85% learning curve
- Simple electronics assembly: 90-95% learning curve (less learning potential)

### 6.2 CleanWalker Learning Curve Projection

**Assumptions:**
- First unit (fully documented prototype): 80 hours
- 80% learning curve through first 100 units
- Plateau at ~25-30 hours after 500 units (diminishing returns)

| Cumulative Units | Average Hrs/Unit | Marginal Hrs (Last Unit) | Reduction from First |
|------------------|------------------|--------------------------|---------------------|
| 1 | 80.0 | 80.0 | 0% |
| 2 | 64.0 | 51.2 | -36% |
| 4 | 51.2 | 41.0 | -49% |
| 8 | 41.0 | 32.8 | -59% |
| 10 | 38.5 | 30.8 | -52% (avg) |
| 20 | 32.8 | 26.2 | -67% (avg) |
| 50 | 26.6 | 21.3 | -73% (avg) |
| 100 | 22.7 | 18.2 | -77% (avg) |
| 500 | 16.0 | 12.8 | -84% (avg) |

**Practical Interpretation for 10-100 Units:**
- First 10 units: 60-70 hrs/unit average
- Units 10-50: 40-50 hrs/unit average
- Units 50-100: 32-42 hrs/unit average
- **Total reduction from 10 to 100 units: ~40-45%**

### 6.3 Factors Affecting Learning Curve

**Accelerators (Steeper Learning Curve):**
- Standardized components (less variation = faster learning)
- Detailed assembly documentation with photos/videos
- Dedicated assembly team (not rotating workers)
- Fixtures and jigs that enforce correct assembly
- Automated testing (removes human variability)

**Inhibitors (Flatter Learning Curve):**
- Design changes mid-production (resets learning)
- High worker turnover (loses institutional knowledge)
- Complex manual tasks with high skill requirement
- Inconsistent component quality (requires troubleshooting)
- Lack of process optimization focus

**CleanWalker Specific:**
- Positive: Standardized actuators (all CubeMars), clear documentation
- Negative: High complexity (12 actuators, weatherproofing), manual wiring harness
- **Expected curve: 80-82% (good but not exceptional)**

### 6.4 Cost Improvement Summary

| Volume Tier | Labor Hrs/Unit | Labor Cost @ $70/hr | Improvement vs 10 Units |
|-------------|----------------|--------------------|-----------------------|
| 10 units | 60 hrs | $4,200 | Baseline |
| 50 units | 42 hrs | $2,940 | -30% |
| 100 units | 35 hrs | $2,450 | -42% |
| 500 units | 24 hrs | $1,680 | -60% |

**Key Takeaway:** The 10-to-100 unit range delivers the steepest cost improvements (40-45% reduction). Diminishing returns after 500 units as you approach theoretical minimum assembly time.

---

## 7. Recommendations

### 7.1 Recommended CM Strategy by Phase

#### Phase 1: Prototype/Pilot (10-20 units)
**Recommended:** Self-assembly or small US CM (PEKO, Fathom)
- **Rationale:** Need tight feedback loop for design iteration; CM won't debug novel designs
- **Cost:** $14,000-$16,000/unit all-in (BOM + assembly)
- **Timeline:** 4-8 weeks per batch

#### Phase 2: Early Production (50-100 units)
**Recommended:** Mid-tier US CM (PEKO, Mack, Benchmark)
- **Rationale:** Assembly procedures proven; ready to scale; US speed advantage still valuable
- **Cost:** $10,000-$11,500/unit all-in
- **Timeline:** 8-12 weeks for first batch, then 6-8 weeks recurring

#### Phase 3: Volume Production (500+ units)
**Recommended:** Large US CM (Jabil, Flex) or EU CM for EU market
- **Rationale:** Cost optimization becomes critical; global supply chain management important
- **Cost:** $7,500-$8,500/unit all-in (US), €7,000-€7,500 (EU)
- **Timeline:** 12-16 weeks first batch, 8-12 weeks recurring

### 7.2 US vs EU Decision Matrix

**Choose US CM if:**
- Fast iteration cycles critical (prototype/pilot phase)
- Team is US-based (communication efficiency)
- Quadruped robotics expertise important
- Target market is North America
- Timeline pressure > cost pressure

**Choose EU CM if:**
- Target market is Europe (tariff avoidance)
- At volume production stage (500+ units)
- Quality/compliance culture critical
- Cost optimization at scale is priority
- Have established EU team/presence

**Hybrid Strategy (Recommended for CleanWalker):**
1. Prototype/pilot: US CM (PEKO/Fathom)
2. EU market entry: EU CM (OMRON Netherlands/Quimesis)
3. Scale production: Dual-source (US + EU) for risk mitigation

### 7.3 Negotiation Leverage Points

**NRE Reduction Strategies:**
- Commit to volume (e.g., "50 units in year 1, 200 in year 2")
- Accept amortization instead of paying upfront
- Provide detailed assembly documentation (reduces CM engineering time)
- Use standard tooling where possible (avoid custom fixtures)

**Per-Unit Cost Reduction:**
- Request learning curve pricing (locked-in reductions at volume milestones)
- Negotiate annual productivity improvements (2-3% YoY)
- Establish long-term partnership (multi-year agreement)
- Assist with supplier relationships (BOM cost reductions benefit both parties)

**Payment Terms:**
- Standard: 50% deposit, 50% on delivery
- Negotiate: 30% deposit, 40% at FAI approval, 30% net-30 after delivery
- At scale: Net-60 or Net-90 terms (requires financial strength)

### 7.4 Red Flags When Evaluating CMs

**Avoid CMs that:**
- Have no robotics assembly experience (learning on your dime)
- Require >50% upfront payment with no milestones
- Won't commit to NRE cap ("time and materials only")
- Can't provide references for similar complexity products
- Have minimum volumes >100 units (not suitable for startup phase)
- Won't sign NDA or IP protection agreements

**Green Flags (Good CM):**
- Willing to build 1-5 pilot units before committing to tooling
- Offers DFM feedback during quoting process
- Has dedicated NPI team (not just production team)
- Transparent pricing breakdown (labor, overhead, materials, margin)
- Offers supply chain support (component sourcing, BOM optimization)
- Flexible on payment terms for startups (milestone-based)

---

## 8. Sources

### Contract Manufacturers
- [Mack Molding Robotics](https://www.mack.com/markets/robotics/) - US robotics CM specialization
- [PEKO Precision Electromechanical Assembly](https://www.pekoprecision.com/contract-assembly-services/electromechanical-assembly/) - US CM capabilities
- [Fathom Manufacturing Robotics](https://fathommfg.com/robotics-industry) - US robotics assembly services
- [OMRON Manufacturing Netherlands](https://industrial.omron.eu/en/our-value/production-base/netherlands) - EU production capabilities
- [VentureOutsource: Comparing Flex, Jabil, Sanmina, Celestica](https://ventureoutsource.com/contract-manufacturing/comparing-flex-jabil-sanmina-celestica-manufacturing-supply-chain-capabilities/) - Large EMS comparison

### Labor Rates & Economics
- [ZipRecruiter: Electromechanical Salary 2026](https://www.ziprecruiter.com/Salaries/Electromechanical-Salary) - US electromechanical wages
- [BLS: US Manufacturing Wages](https://tradingeconomics.com/united-states/wages-in-manufacturing) - $29.51/hr Dec 2025
- [Robin Jobs: Netherlands Production Worker Salary](https://robin.jobs/work-in-the-netherlands/production-worker-salary-in-the-netherlands) - Netherlands wages €13-14/hr
- [Eurostat: EU Hourly Labour Costs](https://ec.europa.eu/eurostat/statistics-explained/index.php/Hourly_labour_costs) - EU average €31.80/hr (2023)
- [Destatis: Germany Manufacturing Labor Costs](https://www.destatis.de/EN/Themes/Countries-Regions/International-Statistics/Data-Topic/Tables/BasicData_LaborCosts.html) - Germany €17/hr

### NRE & Cost Structure
- [PEKO: Contract Manufacturing Costs - NRE, Tooling, Minimum Buys](https://www.pekoprecision.com/blog/contract-manufacturing-costs-nre-tooling-minimum-buys/) - NRE breakdown
- [Averna: NRE Costs Explained](https://insight.averna.com/en/resources/blog/nre-costs-explained) - Testing and validation NRE
- [NAS Electronics: NRE Fees Explained](https://naselectronics.tech/blog-nre-fees) - Electronics NRE categories

### Learning Curves
- [AME: Learning Curves in Manufacturing](https://www.ame.org/sites/default/files/target_articles/01-17-4-Learning_Curve.pdf) - 80% learning curve theory and application
- [Strategose: Learning Curves in Manufacturing](http://www.strategosinc.com/RESOURCES/02-Strategy/learning_curves.htm) - Historical data on learning curves
- [MDPI: Cost Estimating Using Learning Curve Theory](https://www.mdpi.com/2571-9394/2/4/23) - Learning curve for non-constant production rates

### Quality & Testing
- [Spinnaker: Test Development for Contract Manufacturing](https://www.spinnakercontract.com/services/test-development) - Test cost structure
- [CircuitCheck: Impact of Quality on Test Economics](https://www.circuitcheck.com/wp-content/uploads/2019/04/Design_Guide_Cost_Economics_for_Test.pdf) - Functional testing costs
- [ViolinTec: Quality Control in Contract Manufacturing](https://www.violintec.com/contract-manufacturers/quality-control-in-contract-manufacturing-ensuring-product-excellence/) - QA best practices

### Market Research
- [Flex vs Jabil: EMS Stock Analysis 2026](https://www.ainvest.com/news/flex-jabil-ems-stock-offers-risk-adjusted-growth-potential-2026-2512/) - Large EMS market positioning
- [MarkNtel Advisors: US Contract Manufacturing Market Growth to 2032](https://www.marknteladvisors.com/research-library/contract-manufacturing-services-market-us) - Market trends
- [EU Robotics Investment 2026](https://www.assemblymag.com/articles/97914-eu-manufacturers-invest-heavily-in-robotics) - European robotics market

---

## Appendix A: Cost Comparison Table (All Volumes)

| Metric | 10 Units | 50 Units | 100 Units | 500 Units |
|--------|----------|----------|-----------|-----------|
| **US CM - Labor Hrs/Unit** | 60 hrs | 42 hrs | 35 hrs | 24 hrs |
| **US CM - Labor Cost/Unit** | $4,500 | $2,940 | $2,380 | $1,560 |
| **US CM - Total Cost/Unit** | $14,699 | $11,360 | $10,082 | $7,960 |
| **EU CM - Labor Hrs/Unit** | 55 hrs | 38 hrs | 32 hrs | 22 hrs |
| **EU CM - Labor Cost/Unit** | €4,125 | €2,660 | €2,176 | €1,364 |
| **EU CM - Total Cost/Unit** | €13,625 | €10,500 | €9,341 | €7,302 |
| **EU in USD (1.08)** | $14,715 | $11,340 | $10,088 | $7,886 |
| **US vs EU Difference** | -0.1% | +0.2% | -0.1% | +0.9% |

---

## Appendix B: Sample RFQ Template for CMs

When requesting quotes from contract manufacturers, provide:

**1. Product Overview:**
- Product name: CleanWalker Autonomous Litter Collection Robot
- Form factor: Quadrupedal robot, ~800mm length, ~30kg weight
- Complexity: 12 actuators, 2-DOF gripper arm, Jetson Orin compute, LiDAR, cameras, 48V battery
- Assembly type: Electromechanical with IP65 weatherproofing
- Testing: Functional testing (locomotion, gripper, sensors, runtime)

**2. Volume Projections:**
- Year 1: 10-20 units (pilot)
- Year 2: 50-100 units (early production)
- Year 3: 200-500 units (scale production)

**3. Request Breakdown:**
- Per-unit assembly cost at 10, 50, 100, 500 volumes
- NRE estimate (tooling, fixtures, test equipment, engineering support)
- Lead time for first article, first production batch, recurring batches
- Payment terms (deposit, milestone payments, net terms)
- Quality certifications (ISO 9001, etc.)
- References for similar complexity products

**4. BOM Provided:**
- Full component BOM with manufacturer part numbers
- Assembly drawings (mechanical CAD, electrical schematics)
- Assembly procedure documentation (if available)

**5. Timeline:**
- RFQ response requested by: [date]
- First article target: [date]
- First production batch target: [date]

---

*This research document synthesizes data from 20+ sources including contract manufacturers, labor statistics, industry reports, and learning curve theory. All costs are in 2026 USD/EUR. Actual quotes may vary ±15-25% based on specific CM capabilities, current capacity, and negotiation.*

*For CleanWalker decision-making: Use mid-range estimates ($108K NRE, 60 hrs/unit at 10 units, 80% learning curve) as baseline for financial modeling.*
