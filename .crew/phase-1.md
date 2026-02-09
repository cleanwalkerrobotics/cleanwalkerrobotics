# Phase 1 — Cost Discovery Playbook

## Objective

Produce a professional cost model with ±15% confidence that covers all hardware components and contract manufacturing for the robot product. This cost model is the foundation for pricing, deal structuring, and customer pitches.

## Exit Criteria

Phase 1 is complete when ALL of these are met:

- [x] Component-level BOM with specs exists (CTO) — DONE: `docs/hardware-bom-research.md` (prototype $10.2K, production $5.5K)
- [ ] 3+ supplier quotes per major component category (Procurement) — PARTIAL: Research-grade estimates exist, formal RFQs not yet sent
- [ ] 3+ contract manufacturer quotes with cost breakdowns (Manufacturing) — PARTIAL: `docs/assembly-iteration-costs.md` has estimates, formal RFQs not yet sent
- [x] Cost model v1 with unit economics at 3 volume tiers (Finance) — DONE: `docs/pilot-financial-model.md` ($363K investment, $210K revenue)
- [x] Company website is live with product overview (Communications) — DONE: cleanwalkerrobotics.vercel.app deployed on Vercel
- [x] Initial buyer target list with 10+ prospects (BizDev) — DONE: 5 global market research reports identify 10+ cities, 10+ waste companies, and specific grant programs

## Execution Sequence

Phase 1 has dependencies — tasks must flow in the right order. Here's the critical path:

```
Week 1-2: FOUNDATIONS
├── CTO: Produce BOM structure + component requirements
├── Communications: Set up website, brand, email templates
└── BizDev: Begin market research on buyer targets

Week 2-4: OUTREACH
├── Procurement: Send RFQs using CTO's component specs
├── Manufacturing: Send RFQs using CTO's assembly requirements
├── Communications: Finalize website, support outreach templates
└── BizDev: Continue research, build pipeline

Week 4-6: COLLECTION
├── Procurement: Collect and normalize supplier quotes
├── Manufacturing: Collect and normalize CM quotes
├── Finance: Begin building cost model with incoming data
└── BizDev: Finalize target list, prepare for Phase 2 outreach

Week 6-8: SYNTHESIS
├── Finance: Complete cost model v1 with all data
├── CTO: Review cost model for technical accuracy
├── CEO: Review cost model, decide go/no-go for Phase 2
└── BizDev: Refine deal structure based on cost model
```

## Task Breakdown by Role

### CTO — Weeks 1-2 (then on-call)
1. Break robot design into subsystem-level BOM
2. For each component category: spec requirements document (performance, dimensions, interfaces, quantity)
3. Define assembly complexity level and testing requirements for Manufacturing
4. Hand off Component Requirements Doc to Procurement
5. Hand off Assembly Requirements Doc to Manufacturing
6. Remain available for technical questions from Procurement/Manufacturing

### Procurement — Weeks 2-6
1. Receive Component Requirements Doc from CTO
2. Research suppliers per component category (web search, industry databases, Alibaba, direct manufacturer sites)
3. Shortlist 5+ suppliers per category
4. Send RFQs using Communications' email templates
5. Follow up on non-responses (day 5, day 10)
6. Collect quotes, normalize into comparison matrix
7. Submit structured cost data to Finance
8. Flag any components with sourcing risks

### Manufacturing — Weeks 2-6
1. Receive Assembly Requirements Doc from CTO
2. Research contract manufacturers across geographies
3. Shortlist 5+ CMs (mix of domestic, nearshore, offshore)
4. Send RFQ packages with assembly specs
5. Follow up on non-responses
6. Collect quotes with cost breakdowns
7. Submit structured cost data to Finance
8. Produce CM comparison matrix

### Finance — Weeks 4-8
1. Design cost model structure (categories, volume tiers)
2. Ingest Procurement data as it arrives
3. Ingest Manufacturing data as it arrives
4. Build cost model v1 with available data
5. Calculate unit economics at 100, 500, 1000 units
6. Run sensitivity analysis on top cost drivers
7. Recommend target pricing with margin analysis
8. Present cost model to CEO for review

### BizDev — Weeks 1-8 (prep mode)
1. Research government programs related to robotics/automation
2. Research enterprise buyers in target sectors
3. Build prospect list with decision-maker contacts
4. Study procurement processes for top targets
5. Draft outreach templates for Phase 2
6. Define ideal deal structure with Finance (Week 6+)

### Communications — Weeks 1-4 (then support)
1. Set up domain, email accounts, brand identity
2. Build minimum viable website
3. Create RFQ email templates for Procurement and Manufacturing
4. Create professional email signatures
5. Begin pitch deck skeleton
6. Support any material requests from other agents

### CEO — Continuous
1. Week 1: Kick off all agents with initial task assignments
2. Week 2: Check CTO deliverables, ensure Procurement/Manufacturing can start
3. Week 3: Mid-outreach check — any blockers? Adjust priorities
4. Week 4: Review early incoming quotes, flag issues
5. Week 6: Review Finance cost model v1 draft
6. Week 8: Final review, go/no-go decision for Phase 2

## What's Already Done (as of Feb 2026)

### Research & Cost Model
- Full hardware BOM research with subsystem breakdown: `docs/hardware-bom-research.md`
- ML compute costs (training + inference): `docs/ml-compute-costs.md`
- Product specifications + safety requirements: `docs/product-spec-research.md`
- Assembly + iteration costs (60hrs/unit, 3-5 iterations): `docs/assembly-iteration-costs.md`
- Logistics + transport + tariff analysis: `docs/logistics-transport-costs.md`
- Certification requirements for pilot: `docs/certification-pilot-costs.md`
- Complete pilot financial model: `docs/pilot-financial-model.md`

### Market Research (5 regional reports)
- US-focused market research: `docs/market-research.md`
- Europe (EU 27 + UK + CH + NO): `docs/market-research-europe.md`
- Asia-Pacific (10 countries): `docs/market-research-asia-pacific.md`
- MENA + Africa (13 countries): `docs/market-research-mena-africa.md`
- Americas non-US (Canada + LATAM + Caribbean): `docs/market-research-americas.md`
- Global synthesis + competitive landscape: `docs/market-research-global-synthesis.md`

### Infrastructure
- Website live: cleanwalkerrobotics.vercel.app
- Monorepo scaffolded: pnpm + Turborepo + Next.js + Hono + Drizzle + Rust + Python
- CI/CD active, GitHub repo: cleanwalkerrobotics/cleanwalkerrobotics
- CEO knowledge base: `docs/ceo/strategy.md`, `decisions-log.md`, `contacts.md`, `outreach-tracker.md`

### Key Decisions Already Made (9 ADRs)
See `docs/ceo/decisions-log.md` — covers digital-first strategy, AGPL license, Austin US target, $3K-3.5K/mo pricing, tariff mitigation, global market entry (EU+UAE first), Veolia as #1 partner, IFAT Munich as launch event.

## What's Still Needed for Phase 1 → 2 Transition

1. **Formal supplier RFQs** — research-grade estimates exist but no formal quotes from CubeMars, Unitree, etc.
2. **Formal CM RFQs** — assembly cost estimates exist but no contract manufacturer quotes
3. **Pitch deck** — skeleton needed, content available from research reports
4. **Simulation demo** — CAD model + URDF + Gazebo/Isaac Sim video
5. **Perception v1** — YOLO trained on TACO dataset for live AI demo

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| Suppliers don't respond to RFQs | Delays cost model | Cast wide net, follow up aggressively, use multiple channels |
| Component specs too vague for quoting | Inaccurate quotes | CTO provides spec ranges with min/target/max |
| CMs require NDA before quoting | Slows process | Have standard NDA template ready, CEO approves signing |
| Cost model shows product is unaffordable | Kills deal viability | CTO identifies cost-reduction options, Finance models alternatives |
| No clear buyer targets found | No deal pipeline | Broaden search, consider adjacent markets, pivot positioning |
| ~49% tariffs on Chinese actuators | +$24.7K for 10 units | Engage customs broker, check Section 301 exclusions, source non-China |
| IFAT Munich deadline (May 2026) | Miss global launch opportunity | Register startup exhibit area IMMEDIATELY |

## Success Metrics

- **Quote coverage**: % of BOM components with ≥3 quotes
- **Cost confidence**: Range width of cost model (target ±15%)
- **Pipeline size**: Number of identified buyer targets — CURRENTLY 10+ cities, 10+ waste companies
- **Timeline adherence**: Are we on track for 8-week completion?
- **Blocker count**: Number of unresolved blockers at any given time
