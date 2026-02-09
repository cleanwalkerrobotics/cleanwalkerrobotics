# CleanWalker Robotics -- Revised Financial Model

**Date:** 2026-02-09
**Version:** 1.0
**References:** hardware-bom-research.md, assembly-iteration-costs.md, contract-manufacturing-research.md

---

## 1. Unit Economics at Production Scale

### 1.1 Bill of Materials (BOM) by Volume

| Component | 10 units | 100 units | 500 units | 1,000 units |
|-----------|----------|-----------|-----------|-------------|
| Actuators (12x) | $4,800 | $4,360 | $3,800 | $3,500 |
| Gripper Arm | $600 | $480 | $400 | $350 |
| Perception (OAK-D Pro + RPLidar) | $470 | $400 | $350 | $320 |
| Compute (Jetson Orin Nano) | $249 | $280 | $220 | $190 |
| Battery + Power (48V 20Ah) | $380 | $280 | $220 | $180 |
| Compacting Bin | $160 | $100 | $80 | $65 |
| Communications (4G LTE) | $65 | $45 | $35 | $30 |
| PCB / Electronics | $180 | $65 | $45 | $35 |
| Frame + Enclosure | $975 | $512 | $350 | $283 |
| Wiring, connectors, misc | $120 | $80 | $60 | $50 |
| **BOM Subtotal** | **$7,999** | **$6,602** | **$5,560** | **$5,003** |
| Contingency (10%) | $800 | $660 | $556 | $500 |
| **BOM Total** | **$8,799** | **$7,262** | **$6,116** | **$5,503** |

### 1.2 Contract Manufacturing Assembly Costs (US CM)

| Volume | Labor Hrs/Unit | Burdened Rate | Assembly Cost | QA/Test Cost | Total Assembly |
|--------|---------------|---------------|---------------|-------------|----------------|
| 10 units | 60 hrs | $75/hr | $4,500 | $600 | $5,100 |
| 50 units | 42 hrs | $70/hr | $2,940 | $420 | $3,360 |
| 100 units | 35 hrs | $68/hr | $2,380 | $340 | $2,720 |
| 500 units | 24 hrs | $65/hr | $1,560 | $260 | $1,820 |
| 1,000 units | 20 hrs | $62/hr | $1,240 | $200 | $1,440 |

Learning curve: 80% (every doubling of cumulative volume reduces labor by 20%).

### 1.3 Total Manufactured Cost (COGS) Per Unit

| Volume | BOM | Assembly | NRE Amortized | **Total COGS** |
|--------|-----|----------|---------------|----------------|
| 10 units | $8,799 | $5,100 | $800 | **$14,699** |
| 50 units | $7,800 | $3,360 | $200 | **$11,360** |
| 100 units | $7,262 | $2,720 | $100 | **$10,082** |
| 500 units | $6,116 | $1,820 | $40 | **$7,976** |
| 1,000 units | $5,503 | $1,440 | $20 | **$6,963** |

NRE budget: $108,000 (mid-range estimate). Amortized over cumulative production.

---

## 2. RaaS Pricing Model

### 2.1 Tier Structure

| Tier | Monthly Rate | Commitment | Target Customer |
|------|-------------|------------|-----------------|
| **Pilot** | $2,800/mo | 6-month minimum | New customers, proof-of-value |
| **Standard** | $3,500/mo | 12-month contract | Single-site deployments |
| **Fleet** | $3,000/mo | 24-month, 50+ units | Multi-site / city-wide |

- Pilot rate: 20% discount from Standard to reduce adoption friction
- Fleet rate: 14% discount from Standard for volume commitment
- All tiers include robot, software, maintenance, and support

### 2.2 What's Included in Every Tier

- Autonomous litter-collecting robot (hardware)
- Fleet management software + dashboard
- 4G connectivity and real-time telemetry
- Preventive maintenance and field service
- Software/firmware updates (OTA)
- Customer support (business hours)
- Replacement parts and consumables
- Charging dock

---

## 3. Monthly Recurring Costs Per Deployed Unit

### 3.1 Operating Cost Breakdown

| Cost Category | Monthly Cost | Notes |
|---------------|-------------|-------|
| Cellular connectivity (IoT data plan) | $15 | 4G LTE, ~2-5 GB/mo telemetry + low-res video |
| Insurance (commercial liability) | $60 | Per-unit allocation, scales with fleet |
| Preventive maintenance labor | $100 | ~1.5 hrs/mo at $65/hr (field tech at scale) |
| Replacement parts reserve | $75 | Gripper fingers, gaskets, bearings, foot pads |
| Software / cloud infrastructure | $25 | Fleet mgmt hosting, monitoring, OTA platform |
| Customer support allocation | $75 | 1 support specialist per 40 robots |
| **Total Recurring Cost/Unit** | **$350/mo** | |

### 3.2 Gross Margin Per Tier

| Tier | Revenue/mo | Recurring Cost/mo | **Gross Margin/mo** | **Margin %** |
|------|-----------|-------------------|---------------------|-------------|
| Pilot ($2,800) | $2,800 | $350 | $2,450 | 87.5% |
| Standard ($3,500) | $3,500 | $350 | $3,150 | 90.0% |
| Fleet ($3,000) | $3,000 | $350 | $2,650 | 88.3% |

---

## 4. Unit-Level Breakeven Analysis

### 4.1 Months to Recover COGS (Per Unit)

"Breakeven" = months until cumulative gross margin equals the unit's manufactured cost.

| COGS Scenario | Pilot ($2,450/mo margin) | Standard ($3,150/mo margin) | Fleet ($2,650/mo margin) |
|---------------|-------------------------|----------------------------|--------------------------|
| 10 units ($14,699) | 6.0 months | 4.7 months | 5.5 months |
| 50 units ($11,360) | 4.6 months | 3.6 months | 4.3 months |
| 100 units ($10,082) | 4.1 months | 3.2 months | 3.8 months |
| 500 units ($7,976) | 3.3 months | 2.5 months | 3.0 months |
| 1,000 units ($6,963) | 2.8 months | 2.2 months | 2.6 months |

**Key insight:** At $3,500/mo standard pricing, unit-level breakeven occurs within 5 months even at low-volume COGS. At production scale (500+ units), payback is under 3 months on all tiers.

### 4.2 Lifetime Value Per Unit (36-Month Contract Life)

Assuming average contract life of 36 months before major refurbishment:

| COGS Scenario | Tier | 36-mo Revenue | 36-mo Recurring | COGS | **Net Profit/Unit** |
|---------------|------|---------------|-----------------|------|---------------------|
| 100 units | Pilot | $100,800 | $12,600 | $10,082 | **$78,118** |
| 100 units | Standard | $126,000 | $12,600 | $10,082 | **$103,318** |
| 500 units | Fleet | $108,000 | $12,600 | $7,976 | **$87,424** |
| 1,000 units | Standard | $126,000 | $12,600 | $6,963 | **$106,437** |

---

## 5. Fleet-Level Breakeven

### 5.1 Fixed Overhead (Monthly)

These costs are incurred regardless of fleet size once operations begin:

| Category | Monthly Cost | Notes |
|----------|-------------|-------|
| Engineering team (3 FTE) | $45,000 | Software, ML, hardware support |
| Operations manager (1 FTE) | $10,000 | Fleet ops, customer relationships |
| Field technicians (scales) | $0 (included in per-unit) | Already in recurring costs above |
| Office / workshop lease | $4,000 | Co-working or small industrial space |
| Cloud infrastructure (base) | $2,000 | Beyond per-unit allocation |
| Insurance (company-level) | $3,000 | General liability, E&O |
| Legal / accounting | $2,000 | Monthly retainer |
| Sales & marketing | $5,000 | Digital marketing, trade shows |
| Misc / contingency | $4,000 | Travel, equipment, supplies |
| **Total Fixed Overhead** | **$75,000/mo** | **$900,000/year** |

### 5.2 Fleet Breakeven Calculation

Units needed to cover fixed overhead from gross margin:

| Tier | Gross Margin/Unit/Mo | Units to Cover $75K/mo | Units to Cover $75K + 15% Profit |
|------|---------------------|----------------------|----------------------------------|
| Pilot ($2,800) | $2,450 | 31 units | 36 units |
| Standard ($3,500) | $3,150 | 24 units | 28 units |
| Fleet ($3,000) | $2,650 | 29 units | 33 units |
| Blended (~$3,100 avg) | $2,750 | 28 units | 32 units |

**Fleet breakeven at blended rate: ~28 deployed units.**

At a blended average rate of $3,100/mo (mix of tiers), the company reaches cash-flow breakeven at approximately 28 deployed and billing units.

---

## 6. Five-Year Revenue Projections

### 6.1 Deployment Ramp Assumptions

| Year | New Units | Cumulative Deployed | Avg Deployed (for revenue) | Avg Monthly Rate | COGS/Unit |
|------|-----------|--------------------|-----------------------------|-----------------|-----------|
| Y1 | 10 | 10 | 5 | $2,800 (pilot) | $14,699 |
| Y2 | 40 | 50 | 30 | $3,200 (mix) | $11,360 |
| Y3 | 150 | 200 | 125 | $3,300 (mix) | $9,500 |
| Y4 | 300 | 500 | 350 | $3,100 (fleet-heavy) | $7,976 |
| Y5 | 500 | 1,000 | 750 | $3,000 (fleet-heavy) | $6,963 |

Notes:
- "Avg Deployed" accounts for gradual ramp within each year
- Churn assumed at 5% annual after Year 2 (included in cumulative)
- Rate mix shifts toward Fleet tier as volumes grow

### 6.2 Revenue Model

| Year | Avg Units | Avg Rate | **Annual Revenue** | Cumulative Revenue |
|------|-----------|----------|--------------------|--------------------|
| Y1 | 5 | $2,800 | **$168,000** | $168,000 |
| Y2 | 30 | $3,200 | **$1,152,000** | $1,320,000 |
| Y3 | 125 | $3,300 | **$4,950,000** | $6,270,000 |
| Y4 | 350 | $3,100 | **$13,020,000** | $19,290,000 |
| Y5 | 750 | $3,000 | **$27,000,000** | $46,290,000 |

### 6.3 Cost Model

| Year | New Unit COGS | Recurring Ops | Fixed Overhead | **Total Costs** |
|------|--------------|--------------|----------------|-----------------|
| Y1 | $146,990 | $21,000 | $600,000 | **$767,990** |
| Y2 | $454,400 | $126,000 | $900,000 | **$1,480,400** |
| Y3 | $1,425,000 | $525,000 | $1,200,000 | **$3,150,000** |
| Y4 | $2,392,800 | $1,470,000 | $1,800,000 | **$5,662,800** |
| Y5 | $3,481,500 | $3,150,000 | $2,400,000 | **$9,031,500** |

Notes:
- Fixed overhead scales as team grows: $50K/mo Y1 (lean), $75K Y2, $100K Y3, $150K Y4, $200K Y5
- Recurring ops = deployed units x $350/mo x 12
- Y1 reduced overhead reflects pre-revenue startup phase

### 6.4 Profit & Loss Summary

| Year | Revenue | Total Costs | **Net Income** | **Margin** | Cumulative P&L |
|------|---------|-------------|----------------|-----------|----------------|
| Y1 | $168,000 | $767,990 | **-$599,990** | -357% | -$599,990 |
| Y2 | $1,152,000 | $1,480,400 | **-$328,400** | -29% | -$928,390 |
| Y3 | $4,950,000 | $3,150,000 | **+$1,800,000** | +36.4% | +$871,610 |
| Y4 | $13,020,000 | $5,662,800 | **+$7,357,200** | +56.5% | +$8,228,810 |
| Y5 | $27,000,000 | $9,031,500 | **+$17,968,500** | +66.5% | +$26,197,310 |

### 6.5 Milestone Summary

| Milestone | Units | Annual Revenue Run-Rate | Status |
|-----------|-------|------------------------|--------|
| **10 units** | 10 | $336,000 ARR | Pilot validation, product-market fit |
| **50 units** | 50 | $1.86M ARR | Fleet breakeven achieved |
| **200 units** | 200 | $7.92M ARR | Profitable, Series A territory |
| **500 units** | 500 | $18.6M ARR | Scale production, strong margins |

---

## 7. Capital Requirements

### 7.1 Pre-Revenue Investment (Through 10-Unit Pilot)

| Category | Amount | Reference |
|----------|--------|-----------|
| Prototype development (zero to v1.0) | $287,000 | assembly-iteration-costs.md (Recommended Hybrid) |
| NRE for contract manufacturing setup | $108,000 | contract-manufacturing-research.md (mid-range) |
| First 10 pilot units (COGS) | $146,990 | BOM + assembly at 10-unit scale |
| Year 1 operating overhead (12 mo) | $600,000 | Lean team + workspace |
| Certification (FCC, UL 3300) | $250,000 | product-spec-research.md estimate |
| Working capital buffer | $100,000 | Contingency |
| **Total Pre-Revenue Capital** | **~$1.5M** | |

### 7.2 Growth Capital (10 to 50 Units)

| Category | Amount |
|----------|--------|
| 40 new units COGS | $454,400 |
| Scaling team + ops (12 mo) | $900,000 |
| Sales & marketing | $150,000 |
| Working capital | $200,000 |
| **Total Y2 Capital Need** | **~$1.7M** |

### 7.3 Total Funding Requirement

| Phase | Cumulative Funding | Milestone |
|-------|-------------------|-----------|
| Seed / Pre-Seed | $1.5M | 10 pilot units deployed |
| Series A | $3.2M cumulative | 50 units, fleet breakeven |
| Profitability | Self-funding from Y3 | 200+ units |

---

## 8. Sensitivity Analysis

### 8.1 Pricing Sensitivity

| Scenario | Avg Rate | Breakeven Units | Y3 Net Income | Y5 Net Income |
|----------|----------|-----------------|---------------|---------------|
| Bear (15% lower) | $2,635 | 35 units | +$800K | +$12.1M |
| Base | $3,100 | 28 units | +$1.8M | +$18.0M |
| Bull (15% higher) | $3,565 | 23 units | +$2.8M | +$23.8M |

### 8.2 COGS Sensitivity

| Scenario | Impact on Y5 Margin |
|----------|-------------------|
| BOM +20% (supply chain issues) | Y5 margin drops from 52% to 43% |
| BOM -20% (custom actuators) | Y5 margin improves from 52% to 58% |
| Assembly +30% (labor inflation) | Y5 margin drops from 52% to 49% |

### 8.3 Key Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Slow adoption (50% fewer units) | Breakeven delayed to Y4 | Aggressive pilot pricing, guaranteed ROI |
| BOM cost increases (tariffs, supply) | Margin compression 5-10 pts | Dual-source suppliers, buffer inventory |
| High churn (>15% annual) | Revenue plateau | Service SLAs, continuous software improvement |
| Competitive entry | Pricing pressure | First-mover advantage, switching costs, IP |

---

## 9. Key Assumptions

1. **No hardware sales** -- pure RaaS model. CleanWalker retains ownership of all robots.
2. **Robot useful life:** 36 months before major refurbishment (~$2,000 refurb cost).
3. **Contract terms:** 6-month minimum (pilot), 12-month (standard), 24-month (fleet).
4. **Annual churn:** 5% after Year 2 (pilot customers may not all convert).
5. **Deployment capacity:** 1 field tech can support ~40 robots (limiting factor for scale).
6. **No import tariffs** modeled on components (risk if China tariffs escalate).
7. **Currency:** All figures in 2026 USD.
8. **Assembly:** US-based contract manufacturing through Year 3, dual US+EU from Year 4.

---

*This model should be updated quarterly as actual pilot data replaces assumptions. The most uncertain variables are adoption rate and churn -- prioritize tracking these metrics from Day 1 of pilot deployments.*
