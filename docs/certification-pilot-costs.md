# CleanWalker Certification & Compliance: Minimum Viable Pilot Costs

**Date:** February 2026
**Scope:** 10-unit paid pilot on public sidewalks/parks in a US city (San Francisco or Austin)
**Robot:** ~30kg autonomous quadrupedal litter-collecting robot with 4G LTE, WiFi, stereo cameras, LiDAR

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [PDD Classification Analysis](#pdd-classification-analysis)
3. [City-Specific Pilot Requirements](#city-specific-pilot-requirements)
4. [FCC Compliance](#fcc-compliance)
5. [Safety & Liability Insurance](#safety--liability-insurance)
6. [ISO/UL/CE Certification](#isoulce-certification)
7. [Data Privacy (CCPA)](#data-privacy-ccpa)
8. [Lessons from Comparable Companies](#lessons-from-comparable-companies)
9. [Timeline & Phasing](#timeline--phasing)
10. [Cost Summary](#cost-summary)
11. [Recommendation](#recommendation)

---

## Executive Summary

The minimum cost to legally deploy a 10-unit paid pilot in a US city ranges from approximately **$15,000-$45,000** depending on city choice, with Austin, TX being significantly cheaper and faster than San Francisco. The biggest cost items are insurance ($5,000-$15,000/yr), FCC Part 15B SDoC compliance testing (~$3,000-$5,000), legal counsel for city permitting ($5,000-$10,000), and privacy compliance ($2,000-$5,000). Full ISO/UL certification is NOT required for a pilot and can be deferred to commercial launch, saving $50,000-$150,000+.

**Key finding:** Austin is the strongly recommended city for a first pilot. Texas state law (SB 969 / Transportation Code Chapter 552A) preempts local regulation of personal delivery devices and mobile carrying devices, making the regulatory path far more predictable and less expensive than San Francisco's restrictive permit regime.

---

## PDD Classification Analysis

### Does CleanWalker qualify as a Personal Delivery Device (PDD)?

**Short answer: Not exactly, but this is actually an advantage for Austin and a complication for San Francisco.**

PDD laws in both California and Texas define PDDs as devices "manufactured primarily for transporting cargo." CleanWalker collects litter -- it does not deliver goods. This means:

- CleanWalker does NOT fit the strict legal definition of a PDD in most jurisdictions
- There is NO existing regulatory category specifically for autonomous litter-collection robots in either California or Texas
- No autonomous litter-collecting robot has received formal sidewalk approval in any US city (Detroit has deployed autonomous maintenance robots for mowing/plowing/trash pickup, but through municipal programs rather than commercial permits)

### Classification Strategy

**Option A -- Austin (Recommended):** Texas Transportation Code Chapter 552A defines a "personal delivery device" as a device "manufactured primarily for transporting cargo in a pedestrian area." CleanWalker can be argued to transport collected litter (cargo) from pickup points to disposal. Even if this argument is weak, Texas state law preempts local regulation of these devices, and Austin has been permissive with autonomous robots (Avride/Uber Eats launched robot delivery in Austin in late 2024 with minimal friction). The city cannot create new restrictive ordinances due to state preemption. A reasonable legal strategy: classify as a "mobile carrying device" under 552A (which is even more broadly defined) or work directly with Austin Transportation Department for a right-of-way usage authorization for a novel use case.

**Option B -- San Francisco:** SF Public Works Code Section 794 defines "Autonomous Delivery Device" as "a motorized device used to transport items, products, or any other materials on City sidewalks for commercial purposes." CleanWalker's litter collection could potentially be argued as "transporting materials." However, SF's regime is extremely restrictive:
- Maximum 9 devices city-wide (across ALL companies)
- Maximum 3 per applicant
- 3 mph speed limit
- Human operators must be nearby at all times
- Restricted to light-industrial neighborhoods only (130 streets / 761 blocks)
- 180-day permits only
- Parks and trails likely require separate Recreation & Parks Department approval

**Verdict:** Austin offers a dramatically easier regulatory path. San Francisco's 3-device limit alone makes a 10-unit pilot impossible under current rules.

---

## City-Specific Pilot Requirements

### Austin, Texas

#### State-Level Framework (Texas Transportation Code Chapter 552A)

Texas SB 969 (2019) established a statewide framework for personal delivery devices that preempts local regulation:

| Requirement | Detail |
|---|---|
| **Speed limit (sidewalk)** | 10 mph maximum (local authority can reduce to min 7 mph) |
| **Speed limit (roadway shoulder)** | 20 mph maximum |
| **Weight limit** | Less than 110 lbs (50 kg) excluding cargo -- CleanWalker at ~30kg (66 lbs) qualifies |
| **Insurance** | General liability coverage of not less than $100,000 |
| **Human oversight** | A human must have capability to monitor or exercise physical control remotely |
| **Identification** | Marker with owner name, contact info, unique identification number |
| **Braking** | Braking system enabling controlled stop |
| **Lighting (nighttime)** | Front and rear lights visible 1-500 feet |
| **Yield to pedestrians** | Required |
| **Local permits** | City cannot prohibit operation; may set speed minimums (no lower than 7 mph) |

**Austin-specific considerations:**
- Austin City Council approved personal delivery robot projects on sidewalks in 2017
- State law preempts additional city regulation
- Austin Transportation Department handles right-of-way authorization
- No formal city-level permit application process dedicated to PDDs beyond state requirements
- Austin has been actively welcoming to autonomous robots (Avride launched Uber Eats robot delivery in Austin in November 2024)

**Estimated Austin pilot cost for permitting: $2,000-$5,000** (primarily legal review and city coordination)

### San Francisco, California

#### City Permit Framework (Public Works Code Section 794)

| Requirement | Detail |
|---|---|
| **Speed limit** | 3 mph maximum |
| **Max devices per company** | 3 |
| **Max devices city-wide** | 9 |
| **Permit duration** | 180 days |
| **Geographic restriction** | Light-industrial neighborhoods only (130 streets) |
| **Human operator** | Must be nearby at all times |
| **Permit fees** | 1 device: $860; 2 devices: $1,540; 3 devices: $1,995 |
| **Permit extension** | 1: $555; 2: $1,010; 3: $1,465 |
| **Application** | Through SF Public Works Bureau of Street Use & Mapping |

#### California State-Level Issues

- California does NOT have a statewide PDD law comparable to Texas 552A
- Regulation is primarily at the city level, creating a patchwork
- California's autonomous vehicle regulations (DMV) apply to vehicles on roads, not sidewalk robots
- The $5 million insurance/bond requirement applies to autonomous VEHICLES, not sidewalk PDDs (this is the DMV testing permit for road-going AVs)
- For sidewalk-only operations, the state AV regulations likely do not apply

**Critical SF limitations for CleanWalker:**
- Maximum 3 devices per company makes a 10-unit pilot impossible
- 3 mph speed limit is extremely restrictive for coverage area
- Light-industrial neighborhood restriction excludes parks/residential areas
- The definition requires "transporting items for commercial purposes" -- unclear if litter collection qualifies
- Would need separate approval from SF Recreation & Parks for park operations

**Estimated SF pilot cost for permitting: $10,000-$20,000** (legal counsel, application process, potential ordinance amendment lobbying, parks department negotiation) -- and a 10-unit pilot may simply not be possible under current law.

---

## FCC Compliance

### Module Analysis

CleanWalker's RF components:
1. **4G LTE Module (SIM7600G)** -- FCC certified (FCC ID held by SIMCom)
2. **WiFi/Bluetooth (Jetson integrated)** -- FCC certified (FCC ID held by NVIDIA)
3. **No custom RF hardware**

### FCC Obligations When Using Pre-Certified Modules

Per FCC KDB 996369 D04 (Module Integration Guide), when using FCC-certified modular transmitters:

**What you do NOT need:**
- Full FCC certification (Part 22/24/27 for LTE, Part 15C for WiFi/BT)
- FCC ID for the CleanWalker itself (as an intentional radiator)
- Third-party testing for intentional radiator emissions

**What you DO need:**
1. **SDoC (Supplier's Declaration of Conformity) for Part 15 Subpart B (unintentional radiator):** The host product (CleanWalker) contains digital circuitry (Jetson, motor controllers, sensors) that can emit unintentional RF radiation. This requires:
   - EMC testing at a lab per ANSI C63.4
   - Conducted and radiated emissions testing per FCC Part 15.107 and 15.109
   - A self-declaration (no FCC submission needed)
   - The responsible party must be a US-based entity
   - Test report retained on file (submitted to FCC only if requested)

2. **Verify module integration compliance:**
   - Follow each module manufacturer's integration guide
   - Ensure antenna installations match certified configurations
   - Perform limited verification testing of module while installed in host
   - Document compliance

3. **Labeling:**
   - FCC compliance statement on the product
   - Reference to module FCC IDs (can be in manual or e-label)

### FCC Cost Estimate

| Item | Cost | Timeline |
|---|---|---|
| Part 15B EMC testing (SDoC) | $800-$3,000 | 1-3 weeks |
| Module integration verification testing | $1,000-$2,000 | 1-2 weeks |
| FCC compliance consultant (optional) | $1,000-$2,000 | -- |
| **Total** | **$2,800-$7,000** | **2-4 weeks** |

**Key savings:** Because all RF modules (SIM7600G, Jetson WiFi/BT) are pre-certified, we avoid full FCC certification which would cost $10,000-$25,000+ and take 8-12 weeks.

### MUST HAVE for Pilot
- Part 15B SDoC (unintentional radiator compliance) -- legally required before marketing/deployment
- Proper FCC labeling on device
- Module integration documentation

### CAN WAIT
- Nothing -- FCC compliance is required before deployment, but the cost is low with pre-certified modules

---

## Safety & Liability Insurance

### Insurance Requirements

#### Austin (Texas Law)
Texas Transportation Code 552A requires **$100,000 minimum general liability** for PDD operators. This is remarkably low compared to California's AV requirements.

#### San Francisco / California
- The $5 million bond applies to DMV autonomous vehicle testing permits (road-going vehicles), NOT sidewalk PDDs
- SF Section 794 requires "proof of general liability insurance" but does not specify a minimum amount in the publicly available code text
- Cities typically require $1M-$2M general liability in practice

### Insurance Cost Estimates for 10-Unit Pilot

Based on industry data for robotics startups:

| Coverage Type | Coverage Amount | Estimated Annual Premium |
|---|---|---|
| Commercial General Liability (CGL) | $1M per occurrence / $2M aggregate | $2,000-$5,000 |
| Product Liability | $1M-$2M | $3,000-$8,000 |
| Cyber Liability / Data Breach | $1M | $1,500-$3,000 |
| Umbrella / Excess Liability | $5M (if city requires) | $3,000-$7,000 |
| **Total (minimum -- Austin)** | **$100K GL + product liability** | **$3,000-$8,000/yr** |
| **Total (recommended)** | **$1M GL + $1M product** | **$5,000-$13,000/yr** |
| **Total (if $5M required)** | **$5M umbrella** | **$8,000-$20,000/yr** |

**Insurance brokers specializing in robotics:**
- Koop Technologies (koop.ai) -- specialized in AV/robotics insurance, API-based underwriting, claims 10-40% lower premiums than market
- Founder Shield (foundershield.com) -- startup-focused, covers robotics
- Branco Insurance Group -- robotics specialists
- FO Agency -- California robotics & automation venture insurance

**Recommendation:** Start with Koop Technologies for quotes. Their API-driven underwriting approach can provide better rates for robotics companies with good safety data. Budget $5,000-$15,000/year for the pilot.

### Legal Exposure If Robot Injures Someone

- **Product liability** applies -- the manufacturer (CleanWalker) would be the primary defendant
- **Negligence** claims possible against the operating entity
- **Strict liability** may apply in California (even without negligence, manufacturer liable for defective product)
- **Key mitigation:** Maintain remote human monitoring capability (required by Texas law anyway), robust logging/telemetry, and e-stop capability
- **Indemnification:** Cities typically require the company to indemnify the city against all claims arising from robot operations. This is standard and non-negotiable.

### MUST HAVE for Pilot
- General liability insurance meeting city/state minimums ($100K Texas / $1M+ recommended)
- Product liability insurance
- Indemnification agreement with city
- Remote monitoring and e-stop capability

### SHOULD HAVE for Pilot
- Umbrella policy to $5M
- Cyber/data breach coverage
- Workers' compensation (if employees operate in the field)

---

## ISO/UL/CE Certification

### ISO 13482 (Personal Care Robots)

- **What it is:** International standard for safety requirements of personal care robots, including mobile servant robots
- **Required for pilot?** NO. ISO 13482 is a voluntary standard. There is no US federal or state law requiring ISO 13482 certification for sidewalk robots.
- **Cost:** $50,000-$150,000+ for full certification (depends on testing scope, design complexity, and whether components have existing certifications)
- **Timeline:** 6-18 months
- **When to pursue:** Before commercial launch at scale, or when pursuing enterprise/municipal contracts where buyers require it

### UL 3300 (SCIEE Robots)

- **What it is:** ANSI/CAN/UL 3300 is the standard for Service, Communication, Information, Education and Entertainment (SCIEE) Robots. Covers safety of autonomous service robots.
- **Required for pilot?** NO. UL 3300 is a voluntary standard. No US jurisdiction requires it as a condition of deployment.
- **Cost:** Initial assessment $5,000-$10,000; full certification can reach six figures if no components have prior certification. With pre-certified components (motors, batteries, controllers), $30,000-$80,000.
- **Timeline:** 3-12 months depending on design readiness
- **When to pursue:** LG's CLOi ServeBot was the first service robot to achieve UL certification (2022). Getting UL 3300 early provides significant market credibility and may be required by future municipal contracts.
- **Note:** The 2024 edition (ANSI/CAN/UL 3300:2024) was published in April 2025.

### CE Marking

- **Required for pilot?** NO. CE marking is for the EU market only. Not applicable to US deployment.
- **When to pursue:** Only if/when CleanWalker enters EU markets.

### Battery Safety (UL 2271 / UN 38.3)

- **What it is:** UL 2271 covers batteries for light electric vehicle applications; UN 38.3 covers lithium battery transport safety
- **Required for pilot?** UN 38.3 is REQUIRED for shipping lithium batteries (including within the robot). UL 2271 is not strictly required but highly recommended.
- **Cost:** UN 38.3 testing: $3,000-$8,000 per battery model. If using a pre-certified battery pack, this is already covered.
- **Key question:** Is CleanWalker using off-the-shelf battery packs with existing UN 38.3 certification? If yes, no additional testing needed.

### Electrical Safety (UL 60950-1 / UL 62368-1)

- **Required for pilot?** Not strictly legally required for a pilot, but if using certified power supplies and chargers, this is already covered.

### Summary Table

| Standard | Required for Pilot? | Required for Commercial? | Cost | Timeline |
|---|---|---|---|---|
| ISO 13482 | No | Strongly recommended | $50K-$150K | 6-18 months |
| UL 3300 | No | Strongly recommended | $30K-$80K | 3-12 months |
| CE Marking | No (US only) | Only for EU | $20K-$50K | 3-6 months |
| UN 38.3 (battery shipping) | Yes (for shipping) | Yes | $0-$8K* | 2-4 weeks |
| UL 2271 (battery safety) | No | Recommended | $10K-$20K | 2-4 months |

*$0 if using pre-certified battery packs

### MUST HAVE for Pilot
- UN 38.3 battery transport compliance (likely already covered by battery supplier)

### SHOULD HAVE for Pilot
- Begin UL 3300 pre-assessment ($5K-$10K) to identify design issues early

### CAN WAIT until Commercial Launch
- Full ISO 13482 certification
- Full UL 3300 certification
- CE marking (EU only)
- UL 2271 battery safety certification

---

## Data Privacy (CCPA)

### CCPA Obligations for CleanWalker in California

CleanWalker has stereo cameras and LiDAR that will capture images of people on public sidewalks and in parks. Under CCPA:

#### Current Requirements (effective now)

1. **Privacy Policy:** Must have a publicly accessible privacy policy disclosing:
   - Categories of personal information collected (images, video, biometric data if facial recognition used)
   - Purpose of collection
   - How data is shared
   - Consumer rights (access, deletion, opt-out)

2. **Signage Requirements:** If collecting personal information via cameras in physical spaces, CCPA requires **conspicuous signage** informing consumers. The robot should display visible notices such as "This robot is equipped with cameras. For more information visit [URL]."

3. **Data Minimization:** Only collect what is necessary. If cameras are for navigation/obstacle avoidance, ensure face data is not stored or is immediately anonymized.

4. **Consumer Rights:** California residents can request access to, deletion of, and opt-out of sale of their personal information.

5. **Risk Assessment:** Businesses processing personal information for "significant risk" activities must conduct formal risk assessments. Systematic observation via video on public sidewalks qualifies.

#### Upcoming Requirements (2026-2027)

- **January 1, 2026:** Updated CCPA regulations take effect with stricter consent and disclosure requirements
- **January 1, 2027:** New Automated Decision-Making Technology (ADMT) requirements take effect -- if cameras/AI make decisions about consumers, must provide notice and offer opt-out

#### Austin / Texas Considerations

- Texas does not have a comprehensive CCPA equivalent (Texas Data Privacy and Security Act is more limited)
- Texas TDPSA applies to businesses that process data of 100,000+ consumers or derive 50%+ revenue from data sales -- likely does not apply to a 10-unit pilot
- However, following CCPA standards is best practice regardless of jurisdiction

### Privacy Compliance Cost

| Item | Cost |
|---|---|
| Privacy policy drafting (attorney) | $1,500-$3,000 |
| Privacy impact assessment | $2,000-$5,000 |
| Signage/labeling for robots | $200-$500 |
| Data retention policy documentation | $1,000-$2,000 |
| Technical implementation (data anonymization pipeline) | Engineering time |
| **Total** | **$4,700-$10,500** |

### MUST HAVE for Pilot (California)
- Privacy policy
- Conspicuous camera signage on robot
- Data retention and minimization policy
- Consumer rights request process

### MUST HAVE for Pilot (Austin)
- Privacy policy (best practice)
- Camera signage (best practice, builds public trust)

### CAN WAIT
- Full ADMT compliance (not required until 2027)
- Formal cybersecurity audit (new CCPA requirement, not yet in effect)

---

## Lessons from Comparable Companies

### How Starship Technologies Launched

- Started testing in over 100 cities across 20 countries before commercial launch
- First US commercial operations on university campuses (lower regulatory barriers than public streets)
- Worked with individual cities to establish pilot programs
- Secured approval for pilots in Washington D.C. and Redwood City, CA (November 2024)
- Did NOT require ISO/UL certification for initial deployments
- Key strategy: start on private property (campuses) then expand to public sidewalks

### How Serve Robotics (formerly Postmates X) Launched in SF

- Postmates worked with SF Supervisor Norman Yee and advocacy groups to develop the sidewalk robotics framework
- Received SF's first-ever autonomous delivery device permit in August 2019
- Limited to 3 robots for 180 days
- Required nearby human operators at all times
- Permit through SF Public Works Department

### How Kiwibot Launched

- Started on university campuses (UC Berkeley, then expanded)
- Used college campuses as testing grounds (private property, simpler permits)
- Expanded to public streets in cities with permissive PDD laws
- Did not pursue full ISO/UL certification for initial campus deployments

### How Nuro Launched (Road-Going Vehicle -- Different Category)

- Obtained NHTSA exemption for vehicle without traditional safety equipment (no mirrors, no steering wheel)
- Obtained California DMV autonomous vehicle deployment permit (December 2020)
- Required $5 million insurance/bond (this is the DMV AV requirement, not applicable to sidewalk PDDs)
- Required Law Enforcement Interaction Plan
- Required annual technology updates
- Much more intensive regulatory path because Nuro operates on roads, not sidewalks

### Key Takeaway

Every successful sidewalk robot company launched their first pilots either (a) on private property (university campuses) or (b) in cities/states with permissive PDD frameworks. None required ISO/UL certification for initial deployment. Insurance and city permitting were the primary requirements.

---

## Timeline & Phasing

### Phase 1: NOW (Months 1-3) -- Pre-Pilot Preparation

| Task | Timeline | Cost |
|---|---|---|
| Engage robotics-specialized attorney | Week 1-2 | $3,000-$5,000 retainer |
| Determine PDD classification strategy with counsel | Week 2-4 | Included in legal |
| Contact Austin Transportation Department re: pilot | Week 2-4 | $0 |
| Begin FCC Part 15B EMC testing (SDoC) | Week 3-6 | $2,800-$5,000 |
| Obtain insurance quotes (Koop, Founder Shield) | Week 2-4 | $0 |
| Draft privacy policy | Week 4-6 | $1,500-$3,000 |
| Conduct privacy impact assessment | Week 4-8 | $2,000-$5,000 |
| Design and produce camera signage for robots | Week 6-8 | $200-$500 |
| Verify battery UN 38.3 compliance (via supplier) | Week 2-4 | $0-$3,000 |
| Prepare Law Enforcement Interaction Plan | Week 6-10 | $1,000-$2,000 |
| Bind insurance policies | Week 8-10 | $5,000-$15,000/yr |

### Phase 2: DEPLOYMENT (Months 3-4)

| Task | Timeline | Cost |
|---|---|---|
| File right-of-way / pilot authorization with Austin | Week 10-14 | $500-$2,000 |
| Execute indemnification agreement with city | Week 12-14 | Included in legal |
| Install FCC-compliant labeling on all units | Week 12-14 | $100-$300 |
| Final safety review and documentation | Week 12-14 | $1,000-$2,000 |
| Deploy pilot | Week 14-16 | -- |

### Phase 3: DURING PILOT (Months 4-10)

| Task | Timeline | Cost |
|---|---|---|
| Collect safety and operational data | Ongoing | $0 |
| Begin UL 3300 pre-assessment | Month 5-6 | $5,000-$10,000 |
| Refine data privacy practices based on field experience | Ongoing | $0 |
| Monitor regulatory developments | Ongoing | $0 |

### Phase 4: POST-PILOT / COMMERCIAL LAUNCH PREP (Months 10-24)

| Task | Timeline | Cost |
|---|---|---|
| Full UL 3300 certification | 3-12 months | $30,000-$80,000 |
| ISO 13482 certification | 6-18 months | $50,000-$150,000 |
| Expand to additional cities | Ongoing | Variable |
| Scale insurance coverage | As needed | Variable |
| CE marking (if EU launch) | 3-6 months | $20,000-$50,000 |

---

## Cost Summary

### Minimum Cost: 10-Unit Paid Pilot in Austin, TX

| Item | Required for Pilot? | Estimated Cost | Timeline |
|---|---|---|---|
| **Legal counsel (PDD classification, city coordination)** | MUST HAVE | $5,000-$10,000 | 1-3 months |
| **FCC Part 15B SDoC (EMC testing + documentation)** | MUST HAVE | $2,800-$5,000 | 2-4 weeks |
| **FCC module integration verification** | MUST HAVE | $1,000-$2,000 | 1-2 weeks |
| **General liability insurance ($1M)** | MUST HAVE | $2,000-$5,000/yr | 1-2 weeks |
| **Product liability insurance ($1M)** | MUST HAVE | $3,000-$8,000/yr | 1-2 weeks |
| **UN 38.3 battery compliance verification** | MUST HAVE | $0-$3,000* | 2-4 weeks |
| **Privacy policy + impact assessment** | MUST HAVE (CA) / SHOULD HAVE (TX) | $3,500-$8,000 | 2-4 weeks |
| **Camera signage for robots** | SHOULD HAVE | $200-$500 | 1 week |
| **Law enforcement interaction plan** | SHOULD HAVE | $1,000-$2,000 | 2-4 weeks |
| **City pilot authorization / right-of-way** | MUST HAVE | $500-$2,000 | 2-6 weeks |
| **Cyber liability insurance** | SHOULD HAVE | $1,500-$3,000/yr | 1-2 weeks |
| **UL 3300 pre-assessment** | SHOULD HAVE | $5,000-$10,000 | 1-2 months |
| | | | |
| **ISO 13482 full certification** | CAN WAIT | $50,000-$150,000 | 6-18 months |
| **UL 3300 full certification** | CAN WAIT | $30,000-$80,000 | 3-12 months |
| **CE Marking** | CAN WAIT (EU only) | $20,000-$50,000 | 3-6 months |
| **Umbrella insurance to $5M** | CAN WAIT | $3,000-$7,000/yr | 1-2 weeks |

*$0 if battery supplier provides existing UN 38.3 certification

### Total: Minimum to Deploy (MUST HAVE only)

| Category | Low Estimate | High Estimate |
|---|---|---|
| Legal | $5,000 | $10,000 |
| FCC compliance | $3,800 | $7,000 |
| Insurance (Year 1) | $5,000 | $13,000 |
| Privacy compliance | $3,500 | $8,000 |
| Battery compliance | $0 | $3,000 |
| City permitting | $500 | $2,000 |
| **TOTAL MUST HAVE** | **$17,800** | **$43,000** |

### Total: Recommended (MUST HAVE + SHOULD HAVE)

| Category | Low Estimate | High Estimate |
|---|---|---|
| All MUST HAVE items | $17,800 | $43,000 |
| Camera signage | $200 | $500 |
| Law enforcement plan | $1,000 | $2,000 |
| Cyber insurance | $1,500 | $3,000 |
| UL 3300 pre-assessment | $5,000 | $10,000 |
| **TOTAL RECOMMENDED** | **$25,500** | **$58,500** |

### Total: Full Commercial Launch (all certifications)

| Category | Low Estimate | High Estimate |
|---|---|---|
| All recommended items | $25,500 | $58,500 |
| UL 3300 full certification | $30,000 | $80,000 |
| ISO 13482 full certification | $50,000 | $150,000 |
| Umbrella insurance | $3,000 | $7,000 |
| **TOTAL COMMERCIAL** | **$108,500** | **$295,500** |

---

## Recommendation

### Deploy the Pilot in Austin, TX

**Austin is the clear winner for a first pilot.** The reasons:

1. **State preemption:** Texas SB 969 preempts local regulation. Austin cannot impose additional restrictions beyond state law. San Francisco's restrictions (3 devices max per company, 3 mph, industrial areas only) make a 10-unit pilot literally impossible.

2. **Low insurance minimums:** Texas requires only $100,000 GL (though we recommend $1M+). California's AV insurance requirements ($5M) may create confusion even though they likely don't apply to sidewalk PDDs.

3. **Weight compliance:** CleanWalker at ~30kg (66 lbs) is well under Texas's 110 lb limit.

4. **Regulatory precedent:** Avride/Uber Eats launched robot delivery in Austin in November 2024 with robots operating between Cesar Chavez and 15th streets. The city is actively robot-friendly.

5. **Cost:** Total minimum pilot costs in Austin are roughly $15,000-$30,000 versus $25,000-$50,000+ in SF (if even possible).

6. **Novel use case advantage:** Because no litter-collection robot has been deployed in the US before, working with Austin's cooperative regulatory environment is better than fighting SF's restrictive framework. Austin's approach has historically been "work with us" rather than "get a permit or leave."

### Immediate Next Steps

1. **Hire a robotics/autonomous vehicle attorney** licensed in Texas. Budget $5,000-$10,000.
2. **Contact Austin Transportation Department** to introduce the concept and gauge receptivity.
3. **Start FCC Part 15B testing** -- this has no dependency on city choice and takes 2-4 weeks.
4. **Get insurance quotes** from Koop Technologies, Founder Shield, and Branco Insurance Group.
5. **Verify battery UN 38.3 compliance** with your battery supplier.

### Bottom Line

**Minimum cost to legally deploy a 10-unit pilot in Austin, TX: approximately $18,000-$43,000** (including first year of insurance). This is achievable within 3-4 months. Full commercial certification can follow after pilot validation, at an additional $80,000-$230,000 over the following 12-18 months.

---

## Sources & References

- [San Francisco Public Works Code Section 794 - Autonomous Delivery Devices](https://codelibrary.amlegal.com/codes/san_francisco/latest/sf_publicworks/0-0-0-48516)
- [Texas Transportation Code Chapter 552A - Devices Subject to Pedestrian Laws](https://statutes.capitol.texas.gov/Docs/TN/htm/TN.552A.htm)
- [Texas Transportation Code Section 552A.0007 - PDD Equipment Requirements](https://texas.public.law/statutes/tex._transp._code_section_552a.0007)
- [FCC KDB 996369 D04 - Module Integration Guide](https://apps.fcc.gov/kdb/GetAttachment.html?id=bNCiEdkFEKnHsZF9GHCNdg%3D%3D&desc=996369+D04+Module+Integration+Guide+V02&tracking_number=44637)
- [FCC Part 15 Subpart B - Unintentional Radiators](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-B)
- [FCC SDoC Procedure Guide](https://www.compliancegate.com/fcc-sdoc/)
- [UL 3300 Standard for SCIEE Robots](https://www.ul.com/news/ul-3300-outline-investigation-helps-advance-safety-consumer-service-and-education-robots)
- [UL Solutions - Consumer and Commercial Robots](https://www.ul.com/services/consumer-and-commercial-robots)
- [Postmates/Serve Robotics SF Permit (TechCrunch)](https://techcrunch.com/2019/08/07/postmates-lands-first-ever-permit-to-test-sidewalk-delivery-robots-in-san-francisco/)
- [Nuro California DMV Deployment Permit](https://www.dmv.ca.gov/portal/es/news-and-media/dmv-approves-nuro-to-use-autonomous-vehicles-for-commercial-service/)
- [Avride Robot Delivery Austin Launch (KUT)](https://www.kut.org/business/2024-11-15/avride-robots-uber-eats-food-delivery-austin-texas)
- [Detroit Autonomous Urban Maintenance Robots](https://www.thepernateam.com/blog/detroit-leads-the-way-with-autonomous-urban-maintenance-robots/)
- [Koop Technologies - Robotics Insurance](https://www.koop.ai/products/autonomycover/)
- [Founder Shield - Robotics Insurance](https://foundershield.com/industry/robotics/)
- [CCPA Updated Regulations 2025](https://cppa.ca.gov/announcements/2025/20250923.html)
- [Regulating Sidewalk Delivery Robots (Academic)](https://www.tandfonline.com/doi/full/10.1080/02723638.2023.2275426)
- [US States That Have Legalized PDDs](https://www.qmul.ac.uk/decentering-human/media/law/docs/research/US-States-That-Have-Legalized-Personal-Delivery-Devices-or-Last-mile-Autonomous-Delivery-Robots-2020.pdf)
- [Austin Autonomous Vehicles Page](https://www.austintexas.gov/page/autonomous-vehicles)
- [California Autonomous Vehicle Regulations (DMV)](https://www.dmv.ca.gov/portal/vehicle-industry-services/autonomous-vehicles/california-autonomous-vehicle-regulations/)
- [Austin PDD Policy Document](https://services.austintexas.gov/edims/document.cfm?id=389360)

---

*This document is for planning purposes and does not constitute legal advice. Consult with a licensed attorney specializing in autonomous vehicle / robotics law before deployment.*
