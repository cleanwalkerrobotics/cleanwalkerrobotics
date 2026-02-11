# Netherlands Grant & Funding Strategy — CleanWalker Robotics

**Date:** 2026-02-11
**Entity:** MB Software Studio LLC (US LLC, Dutch founder, NL-based operations)
**Stage:** Pre-revenue, digital prototype (ML model, simulation, CAD, URDF)
**Funding Need:** ~€340K to reach pilot stage

---

## Executive Summary

The Netherlands offers several R&D incentive programs that align well with CleanWalker's technology development. This document consolidates all NL-specific funding options, assesses our eligibility, and provides an execution roadmap.

**Key finding:** The LLC-as-BV tax treatment (effective Jan 2025) gives us access to most Dutch incentives, but **converting to a Dutch BV removes all ambiguity** and is strongly recommended before applying for major programs (cost: €500-2,000).

### Priority Matrix

| Priority | Program | Amount | Eligibility | Timeline | Action |
|----------|---------|--------|-------------|----------|--------|
| **1 (NOW)** | WBSO R&D Tax Credit | 50% of R&D wages (starter) | HIGH | Apply now → monthly rolling | Apply for next period ASAP |
| **2 (Apr 2026)** | MIT Haalbaarheid | Up to €20K | HIGH | April 8, 2026 opening | Prepare application now |
| **3 (H2 2026)** | VFF Proof-of-Concept | Up to €450K loan | MEDIUM (needs investor LOI) | Year-round, budget limited | Start investor conversations |
| **4 (H2 2026)** | Innovatiekrediet | €150K-€10M loan | MEDIUM (needs co-financing) | Year-round | Build toward application |
| **5 (When buying HW)** | MIA/Vamil | 27-45% tax deduction on green investment | HIGH (when purchasing) | Within 3 months of purchase | Apply when buying hardware |
| **6 (If relevant challenge)** | Innovation Impact Challenge | Variable | HIGH (if challenge opens) | Monitor | Watch RVO for urban cleaning calls |
| **7 (Equity)** | Regional Development Agency (ROM) | €100K-€500K equity | MEDIUM | Year-round | Introductory meeting |

---

## 1. WBSO (R&D Tax Credit) — HIGHEST PRIORITY

### What It Is

The WBSO (Wet Bevordering Speur- en Ontwikkelingswerk) is the Netherlands' primary R&D tax incentive. It reduces payroll tax for companies performing qualifying R&D, or provides a fixed deduction for self-employed innovators.

### 2026 Rates

| Category | Rate | Threshold |
|----------|------|-----------|
| **Companies — Starter** | **50%** of R&D wage costs | First €391,020 |
| Companies — Regular | 36% of R&D wage costs | First €391,020 |
| All companies | 16% of R&D wage costs | Above €391,020 |
| **Self-employed — Starter** | **€23,975 deduction** | €15,979 base + €7,996 starter supplement |
| Self-employed — Regular | €15,979 deduction | Fixed annual amount |

> **Starter status:** Available for the first 3 applications within the first 5 years of entrepreneurship. We qualify.

### Eligibility Assessment for CleanWalker

| Requirement | Our Status | Notes |
|-------------|-----------|-------|
| Dutch entity subject to payroll/corporate tax | ✅ LLC treated as BV since Jan 2025 | Verify with tax advisor |
| R&D work performed within the EU | ✅ Founder based in Netherlands | Remote work from NL counts |
| Technical research with "technical uncertainties" | ✅ Robotics perception, RL locomotion, grasp planning | See WBSO application draft |
| Minimum 500 R&D hours/year (self-employed only) | ✅ Planning 2,000 hours/year | Well above minimum |
| Apply BEFORE R&D period starts | ⚠️ Apply ASAP for next period | Cannot apply retroactively |
| eHerkenning Level 3 | ❓ Need to obtain | ~€30-50/year |

### Financial Impact Scenarios

**Scenario A: Founder draws €96,000/year salary (DGA/director salary)**

| Item | Amount |
|------|--------|
| R&D wages eligible | €96,000 |
| WBSO benefit (50% starter rate) | **€48,000/year** |
| Effective salary cost after WBSO | €48,000 |

**Scenario B: Founder classified as IB-ondernemer (self-employed)**

| Item | Amount |
|------|--------|
| S&O deduction from profit | €15,979 |
| Starter supplement | €7,996 |
| Total profit deduction | €23,975 |
| Tax saving (~37% rate) | **~€8,870/year** |

> **Recommendation:** Scenario A (salary) provides ~5x more benefit. Discuss with tax advisor whether to structure the LLC with a DGA salary for WBSO purposes.

### Application Timeline

| Action | Deadline | Notes |
|--------|----------|-------|
| Obtain eHerkenning | ASAP | Required for RVO portal access |
| Confirm tax classification | ASAP | Inhoudingsplichtige vs IB-ondernemer |
| Submit WBSO application | End of current month | Period starts 1st of following month |
| Last 2026 application | September 30, 2026 | For October-December period |

### Strategic Value Beyond Tax Savings

- **Quality mark:** WBSO approval signals to investors and other grant bodies that your R&D is validated by the Dutch government
- **Prerequisite:** WBSO is often a soft prerequisite for Innovatiekrediet applications
- **Track record:** Multiple approved WBSO periods build a track record for VFF and EIC applications

**Detailed application draft:** See `wbso-application-draft.md` in this directory.

---

## 2. MIT Haalbaarheidsprojecten (Feasibility Studies)

### What It Is

The MIT (Mkb-innovatiestimulering Regio en Topsectoren) Feasibility Study subsidy covers 35% of costs for researching the technical and economic feasibility of an innovation project. Since 2024, MIT Haalbaarheid is administered by **regional provinces**, not RVO.

### 2026 Details

| Parameter | Value |
|-----------|-------|
| **Subsidy rate** | 35% of eligible costs |
| **Maximum** | €20,000 |
| **Minimum project cost** | ~€57,143 (to reach €20K at 35%) |
| **Duration** | Maximum 12 months |
| **Eligible costs** | Employee wages, external advisory/consultancy, materials, equipment rental |
| **Application** | Through your province's subsidy portal |
| **Mechanism** | First-come-first-served; lottery on day 1 if oversubscribed |

### 2026 Opening Dates by Region

| Province | Opening Date | Notes |
|----------|-------------|-------|
| South Holland | June 9, 2026 — September 15, 2026 | Longer window |
| North Holland | April 8, 2026 (expected) | Typically oversubscribed day 1 |
| Utrecht | April 8, 2026 (expected) | |
| Other provinces | April 8, 2026 (most) | Check provincial websites |

> **Critical:** Budget is typically exhausted on day 1. Prepare the full application in advance and submit the moment the portal opens.

### What to Apply For

**Feasibility Study: Autonomous Quadrupedal Litter Collection for Dutch Municipalities**

Study objectives:
1. Technical feasibility of edge-deployed perception pipeline for Dutch outdoor environments
2. Economic feasibility of RaaS model for Dutch municipal cleaning operations
3. Regulatory pathway assessment under EU Machinery Regulation 2023/1230
4. Pilot site assessment with Amsterdam "Schoon & Afvalvrij" program or Rotterdam circular economy initiative

### Budget Example

| Cost Item | Amount | 35% Subsidy |
|-----------|--------|-------------|
| External robotics engineering consultant (100 hrs × €150) | €15,000 | €5,250 |
| External regulatory assessment (compliance consultant) | €8,000 | €2,800 |
| ML training compute and data costs | €3,000 | €1,050 |
| Hardware for feasibility tests (sensors, dev kit) | €5,000 | €1,750 |
| Market research and customer interviews | €5,000 | €1,750 |
| Personnel costs (own R&D hours) | €21,143 | €7,400 |
| **Total** | **€57,143** | **€20,000** |

### Eligibility Considerations

| Requirement | Our Status | Notes |
|-------------|-----------|-------|
| SME (< 250 employees, < €50M revenue) | ✅ | Solo founder |
| Registered at KvK (Chamber of Commerce) | ⚠️ | LLC should be registered; verify |
| Innovating in a "topsector" area | ✅ | High tech, ICT, circular economy |
| Project not yet started | ✅ | Plan application for April/June |

### MIT R&D Samenwerkingsprojecten (Collaboration Projects)

MIT also offers R&D collaboration grants for projects involving **at least 2 SME partners** from the same region:

| Parameter | Value |
|-----------|-------|
| **Subsidy** | 35% of eligible costs |
| **Maximum** | €200,000 per project (shared among partners) |
| **Duration** | Up to 2 years |
| **Requirement** | Minimum 2 independent SME partners |
| **Application** | Through RVO (national level) |
| **2026 opening** | Expected September 2026 |

**Potential partners for collaboration:**
- Dutch robotics companies (Lely, Demcon, Smart Robotics)
- Dutch AI startups working on edge deployment
- Dutch cleantech companies working on circular economy

> **Not immediately actionable** — requires finding a partner SME. Keep on radar for September 2026.

---

## 3. VFF (Vroegefasefinanciering) — Proof-of-Concept

### What It Is

Government loan specifically designed for taking a concept to a working prototype. The government matches private investor commitment up to €450K.

### 2026 Details

| Parameter | Value |
|-----------|-------|
| **Loan amount** | Up to €450,000 |
| **Interest** | Favorable government rate |
| **Repayment** | Only if project succeeds |
| **2026 budget** | €2.25M (national track) + €4.95M (academic track) |
| **Application** | Year-round through RVO (until budget depleted) |
| **Key requirement** | **Matching investor letter of intent** for at least the same amount |

### Why This Is Transformative

€450K VFF + €450K investor = **€900K total** — enough for multiple prototype iterations, a full 10-unit pilot, and early operations. The government backing also de-risks the deal for the investor.

### Eligibility

| Requirement | Our Status | Notes |
|-------------|-----------|-------|
| Innovative startup < 5 years old | ✅ | |
| Matching investor declaration of intent | ❌ | Need to find angel/VC willing to commit €450K |
| SME classification | ✅ | |
| Clear development plan | ✅ | Detailed pilot financial model exists |

### Action Plan

1. **Start investor conversations now** — Dutch angel networks: Leapfunder, DOEN Foundation, Techstars Amsterdam alumni
2. **Secure an investor Letter of Intent** for at least €200K (VFF scales to match)
3. **Apply immediately** once LOI is secured — budget is very limited (€2.25M national)
4. **Use WBSO approval** as credibility signal in investor conversations

---

## 4. Innovatiekrediet (Innovation Credit)

### What It Is

Government-backed innovation loan (€150K-€10M) for developing technically new products. Repayable only if the project succeeds — essentially risk-free funding for the company.

### Key Conditions

| Requirement | Our Status | Notes |
|-------------|-----------|-------|
| Technically new product (new to NL) | ✅ | No commercial litter-collecting quadruped exists |
| Substantial technical risk | ✅ | Outdoor robotics + AI + manipulation |
| NL-established company with substantial NL activities | ✅ | |
| Market-ready outcome within 5 years | ✅ | Target pilot in 18 months |
| Prior proof-of-concept work | ⚠️ | Software/simulation only — may need more |
| **55% co-financing from other sources** | ❌ | Need ~€200K from investors |
| Complete financing plan | ⚠️ | Need investor term sheets |
| Contingency reserves | ⚠️ | Need operational runway |

### Timeline

- **Not ready now** — need proof-of-concept and co-financing plan
- **Target: H2 2026** after securing WBSO + some investor interest
- **Having WBSO approval strengthens this application significantly**

---

## 5. MIA/Vamil (Environmental Investment Deductions)

### What It Is

Tax deductions for investments in environmentally beneficial equipment. MIA provides 27%, 36%, or 45% extra deduction on qualifying investments. Vamil allows 75% accelerated depreciation.

### When It Becomes Relevant

When we start purchasing hardware for prototypes:

| Purchase | Estimated Cost | MIA Deduction (27-45%) | Tax Saving |
|----------|---------------|----------------------|------------|
| Jetson Orin Nano Super | €249 | €67-112 | €17-29 |
| OAK-D Pro camera | €399 | €108-180 | €28-47 |
| Livox Mid-360 LiDAR | €749 | €202-337 | €53-87 |
| CubeMars actuators (12x) | ~€5,000 | €1,350-2,250 | €351-585 |
| Full prototype BOM | ~€10,224 | €2,760-4,601 | €718-1,196 |

> **Requirement:** Must notify RVO within 3 months of the investment. The equipment must be on the "Milieulijst" (Environment List) or qualify under generic categories for waste-reduction technology.

---

## 6. Innovation Impact Challenge (Dutch SBIR)

### What It Is

Government procurement program where Dutch ministries define innovation challenges and invite companies to propose solutions. Phase 1 funds feasibility research; Phase 2 funds development to end product.

### Why It Matters

If the Ministry of Infrastructure and Water Management (I&W) or a municipality issues a challenge related to **autonomous urban cleaning, smart waste management, or public space maintenance**, this is an ideal fit.

### Current Status

No relevant open calls as of February 2026. However, this program is worth monitoring and proactively engaging with:

**Action:** Contact RVO to suggest a challenge in this domain. The Dutch government's focus on circular economy and smart cities makes an autonomous cleaning challenge plausible.

---

## 7. Regional Development Agencies (ROMs)

### What They Are

Provincial equity investment agencies that co-invest alongside private investors in innovative companies. NOT grants — they take equity positions.

### Relevant ROMs

| Province | ROM | Focus Areas | Contact |
|----------|-----|-------------|---------|
| South Holland | InnovationQuarter | Deep tech, cleantech, maritime | innovationquarter.nl |
| North Holland | ROM InWest | SME Fund, Transition Fund | romregion.nl |
| Noord-Brabant | BOM | High tech, smart industry | bom.nl |
| Overijssel/Gelderland | Oost NL | High tech systems | oostnl.nl |

### How They Work

- Typical investment: €100K-€500K equity
- Usually co-invest alongside private investors
- Can provide introductions to regional ecosystem
- Robotics + cleantech is in scope for most ROMs

### Action

Identify which province CleanWalker operates from and schedule an introductory meeting with the relevant ROM. They can also help connect to other funding sources and ecosystem partners.

---

## 8. Other NL Opportunities (Lower Priority)

### Techleap (Rise / Pole Position)

- Non-profit subsidized by Ministry of Economic Affairs
- Rise: 8-week sector-agnostic scaleup program
- Pole Position: Deep-tech specific verticals
- Value: Mentorship, network, investor connections
- **Fit: 5/10** — more suited to companies with initial traction

### Eurostars (International R&D Collaboration)

- Up to €500K for NL share of international R&D project
- Requires partner from another Eurostars country
- **Deadline: March 19, 2026** — tight but possible if partner available
- **Fit: 4/10** — requires finding international partner quickly

### Municipal Innovation Programs

Not grants, but potential pilot partnerships:

| City | Program | Opportunity |
|------|---------|-------------|
| Amsterdam | "Schoon & Afvalvrij" Framework 2025-2028 | Pilot city partner |
| Rotterdam | Circular economy 2030 goals | Pilot city partner |
| The Hague | Smart city initiatives | Pilot city partner |
| Utrecht | Healthy urban living focus | Pilot city partner |

---

## 9. Legal Structure Recommendation

### Current: US LLC (MB Software Studio LLC)

Since January 1, 2025, Dutch tax law treats a US LLC as a non-transparent entity equivalent to a BV, subject to Dutch corporate income tax. This gives us access to most Dutch incentives.

### Recommended: Convert to Dutch BV

| Benefit | Details |
|---------|---------|
| Removes all eligibility ambiguity | BV is the standard NL entity — no questions from RVO, investors, or partners |
| Eliminates double-taxation risk | LLC is transparent for US tax (pass-through) but opaque for NL tax (corporate) — creates conflicts |
| Expected by EU investors | VCs and angels expect a BV structure |
| Costs €500-2,000 | One-time incorporation cost |
| Standard for NL tech startups | Every NL startup uses BV |

### Conversion Timing

| Action | When | Estimated Cost |
|--------|------|---------------|
| Consult Dutch tax advisor on LLC vs BV | **This week** | €200-500 (initial consultation) |
| If converting: notary for BV incorporation | Within 2-4 weeks | €500-1,500 |
| Transfer activities from LLC to BV | After incorporation | €200-500 (legal/admin) |
| Update KvK registration | Same day as incorporation | Free |

> **CRITICAL:** Converting BEFORE applying for WBSO, MIT, and VFF removes all risk of entity-related rejection. The cost (€500-2,000) is negligible compared to the potential grant/tax benefits (€20K-€450K+).

---

## 10. Execution Roadmap

### Phase 1: Immediate (February-March 2026)

| # | Action | Owner | Deadline |
|---|--------|-------|----------|
| 1 | Consult Dutch tax advisor on LLC vs BV + WBSO classification | Founder | This week |
| 2 | Obtain eHerkenning Level 3 | Founder | Within 2 weeks |
| 3 | Submit WBSO application for next available period | Founder | End of Feb/March |
| 4 | Set up R&D hour tracking system | Founder | Before WBSO period starts |
| 5 | Begin preparing MIT Haalbaarheid application | Founder | March 2026 |

### Phase 2: Short-term (April-June 2026)

| # | Action | Owner | Deadline |
|---|--------|-------|----------|
| 6 | Submit MIT Haalbaarheid application on opening day | Founder | April 8, 2026 |
| 7 | Schedule ROM introductory meeting | Founder | April 2026 |
| 8 | Begin investor conversations for VFF matching | Founder | April-June 2026 |
| 9 | Contact Amsterdam "Schoon & Afvalvrij" team for pilot partnership | Founder | May 2026 |

### Phase 3: Medium-term (July-December 2026)

| # | Action | Owner | Deadline |
|---|--------|-------|----------|
| 10 | Apply for VFF when investor LOI secured | Founder | When ready |
| 11 | Explore MIT R&D Samenwerkingsprojecten with partner | Founder | September 2026 opening |
| 12 | Build toward Innovatiekrediet application | Founder | H2 2026 |
| 13 | Apply for MIA/Vamil when purchasing first hardware | Founder | Within 3 months of purchase |

### Phase 4: Long-term (2027+)

| # | Action | Timeline |
|---|--------|----------|
| 14 | EIC Physical AI Challenge (with prototype + municipal LOI) | Q1-Q2 2027 |
| 15 | EIC Accelerator Open (when at TRL 6+ with revenue) | Q3-Q4 2027 |
| 16 | Innovatiekrediet full application (with co-financing secured) | 2027 |

---

## 11. Cumulative Funding Potential

### Best-Case Scenario (All Programs Secured)

| Program | Amount | Type | Timeline |
|---------|--------|------|----------|
| WBSO (Year 1) | €48,000 | Tax credit | 2026 |
| MIT Haalbaarheid | €20,000 | Grant | 2026 |
| VFF | €450,000 | Loan (success-dependent) | 2026-2027 |
| MIA/Vamil (on €50K hardware) | €13,500-€22,500 | Tax deduction | When purchasing |
| **Subtotal NL funding** | **€531,500-€540,500** | | |
| Plus: Matching investor capital (for VFF) | €450,000 | Equity | 2026-2027 |
| **Total available capital** | **~€981,500-€990,500** | | |

This exceeds our estimated ~€340K funding need for reaching pilot stage, with significant runway for multiple prototype iterations and early operations.

### Realistic Scenario

| Program | Amount | Probability |
|---------|--------|------------|
| WBSO | €48,000 | 85% (if tax structure is correct) |
| MIT Haalbaarheid | €20,000 | 50% (lottery risk on day 1) |
| VFF | €200,000 | 30% (depends on investor) |
| MIA/Vamil | €5,000 | 90% (straightforward) |
| **Expected value** | **~€110,000** | |

Even the realistic scenario provides meaningful funding, and the WBSO approval builds credibility for all subsequent applications.

---

## Sources

### Official Government Sources
- [RVO.nl — WBSO](https://www.rvo.nl/subsidies-financiering/wbso)
- [RVO.nl — MIT](https://www.rvo.nl/subsidies-financiering/mit)
- [RVO.nl — Innovatiekrediet](https://www.rvo.nl/subsidies-financiering/innovatiekrediet)
- [RVO.nl — VFF](https://www.rvo.nl/subsidies-financiering/vroegefasefinanciering)
- [RVO.nl — MIA/Vamil](https://www.rvo.nl/subsidies-financiering/mia-vamil)
- [RVO.nl — Innovation Impact Challenge](https://www.rvo.nl/subsidies-financiering/innovatie-impact-challenge)
- [Business.gov.nl — WBSO](https://business.gov.nl/subsidies-and-schemes/wbso/)
- [Ondernemersplein — MIT](https://ondernemersplein.overheid.nl/subsidies-en-regelingen/mkb-innovatiestimulering-regio-en-topsectoren/)

### Provincial MIT Sources
- [Provincie Zuid-Holland — MIT Haalbaarheidsprojecten](https://www.zuid-holland.nl/online-regelen/subsidies/subsidies/mkb-subsidies/mkb-haalbaarheidsprojecten-mit/)
- [Provincie Noord-Holland — MIT](https://www.noord-holland.nl/Producten_op_alfabet/MKB_innovatiestimulering_topsectoren_MIT_Noord_Holland_Haalbaarheidsproject_subsidie)

### Advisory Sources
- [Hezelburcht — WBSO 2026](https://www.hezelburcht.com/nieuws/wbso-2025-gemist-dit-is-de-planning-met-bijbehorende-deadlines-voor-wbso-2026/)
- [Fiscount — WBSO 2026](https://www.fiscount.nl/publicaties/subsidie/wbso-2026-deadlines-en-budget-ongewijzigd/)
- [Subsidium — WBSO 2026](https://subsidium.nl/wbso-2026/)
- [Digital Holland — MIT subsidiewijzer](https://digital-holland.nl/subsidiewijzer/nationale-subsidie-instrumenten/mit)

---

*Compiled by cw-bizdev team, 2026-02-11. All deadlines and rates verified against official RVO.nl sources and Dutch government publications. Verify all deadlines directly with administering bodies before applying. Consult a Dutch tax advisor (e.g., Broadstreet, Stibbe, or local belastingadviseur) before submitting any applications.*
