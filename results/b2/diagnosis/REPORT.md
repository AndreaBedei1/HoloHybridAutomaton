# Diagnosi dei run non-PASS della campagna B0/B2 (b2_final, parziale)

Stato al 2026-10-29. Campagna `b2_final` (429 run) fermata a mano: 321 run validi conservati in
`results/robustness/raw_b2`, 105 INVALID di infrastruttura e 3 run interrotti (archiviati in
`results/robustness/raw_b2_interrupted_20261029_1404`), 108 run ancora da fare. In questa fase non è stato modificato
codice di controllo, percezione, automa, soglie, rumore o deadline; gli script sotto `scripts/` sono solo di analisi e
leggono i log grezzi (osservazioni onboard, transizioni dell'automa, eventi, serie del referee).

## 0. Integrità e confrontabilità dei dati

| Verifica | Esito |
|---|---|
| Run validi | 321: 273 PASS, 46 SAFE_DEGRADED, 2 LIVENESS_FAILURE, 0 SAFETY_FAILURE |
| Sicurezza (tutti i 321) | 0 contatti, 0 false release del gate, P2 rispettata 321/321 (occupazione massima 1), separazione vera minima 2.258 m (d_safe 1.0 m) |
| INVALID (105) | tutti al commit `3992df2`: lancio parallelo con orchestratori orfani, worker che non partono (exit 0xC0000142). Infrastruttura, nessun contenuto scientifico. Si rifanno al prossimo lancio |
| Versioni del codice, B0 | run a `8aacabc`, `a1bed79`, `1838b65`, `3992df2`, `a4701fa`. I cambiamenti intermedi toccano solo il ramo acustico (gate di slot e regola A-P attivi solo con fix), le identità del beacon (solo con beacon attivo) e un contatore informativo del referee: il percorso B0 è equivalente |
| Versioni del codice, B2 | tutti i B2 validi sono a codice corretto (>= `27ed79c`); tutti i B2 di `p1_two_lines` hanno la correzione delle identità per formazione (`a4701fa`) |
| Riproducibilità | i run NON sono riproducibili passo per passo: due B0 CLEAN con lo stesso seed (campagne diverse) differiscono già al primo passo (nav_p a livello del mm-cm). Il seed fissa le finestre di guasto pianificate (identiche fra B0 e B2: stesse entrate in FAILSAFE), non la realizzazione del rumore (`HoloFleetSim.nrng` è un unico stream consumato nell'ordine delle letture) né la fisica del motore |
| Conseguenza | B0 e B2 con lo stesso seed sono realizzazioni indipendenti. Con 3 seed per cella, differenze di 1-2 run per cella non sono significative; i confronti sono statistici |
| Celle mancanti | `line_parallel_mutex`, `lost_drone_mutex`, `mutex_deadlock_resolution` in LOW/MEDIUM (B0 e B2) e `integrated_short` con canale BLACKOUT sono quasi tutti INVALID: nessun dato su questi casi |

## 1. Quadro delle cause

I 48 run non-PASS (46 SAFE_DEGRADED + 2 LIVENESS_FAILURE) finiscono tutti alla DEADLINE senza violazioni di
sicurezza. SAFE_DEGRADED e LIVENESS_FAILURE differiscono solo per la presenza di un evento "conservativo"
riconosciuto dal classificatore (falsa vacanza, o FAILSAFE da guasto sensore su un drone sano), non per la causa del
mancato completamento (vedi C6).

| Causa | Tipo | Run non-PASS (primaria / concausa) | B0 vs B2 | Correggibile |
|---|---|---|---|---|
| C1 falso SEPARATION_WARNING da target multi-settore agli "angoli" della formazione | difetto strutturale di progetto (percezione vs soglie) | 24 (22 da sola) / +15 | identica (livello sonar) | sì, in percezione |
| C2 deriva di navigazione lungo-traccia non corretta | limite di osservabilità di B0 | 6 / +15 (+3 gate tramite C5) | B2 col canale la riduce; B2 BLACKOUT = B0 | no in B0 (B2 è la soluzione) |
| C3 ambiguità di associazione eco-slot (solo distanza) | limite del sonar a fascio largo, aggravato da C1 | 0 / 12 (revoca della vacanza) + 8 (false vacanze) | presente in entrambi; in B2 le fix non aiutano quando mancano | in parte, in B2 |
| C4 echi fantasma vicini (rumore d'intensità MEDIUM) | comportamento conservativo, limite rumore/sensore | 0 / diffusa nei MEDIUM | identica | solo toccando la prova P1: non ora |
| C5 gate: deriva → barra non spiegata → ostacolo; aiding attivo solo entro 2.0 m | difetto strutturale di progetto | 3 | identica | sì, in structure_aiding |
| C6 classe LIVENESS/SAFE_DEGRADED non legata alla causa | limite del classificatore | (2 LIVENESS) | - | solo come tag diagnostico aggiuntivo |

## 2. Cause confermate ed evidenze

### C1 - Falso SEPARATION_WARNING da target multi-settore (formazioni quadrato e tee)

**Meccanismo.** `perception/targets.build_targets` unisce gli echi di settori adiacenti con distanze entro
`assoc_m` = 0.6 m in un unico target. Per il pattern risultante (es. LEFT+REAR) il centro non è certificato nel cono di
nessun membro, quindi `sonar_geometry.target_distance_lower` usa il bound (B) invece di (A) per entrambi: ~0.5 m più
basso. In un quadrato a 3.5 m ogni drone vede due vicini a 90° (es. sinistro e posteriore) con echi a 2.92 e 3.07 m:
vengono fusi in un target LEFT+REAR con d_lower ~2.5 m invece di ~2.9-3.0 m.

**Evidenze.**
- Su tutti i 321 run: 5401 ingressi in SEPARATION_WARNING; **nessuno** con distanza vera < 2.4 m; 4241 (78.5%) da
  target multi-settore, di cui 4203 con distanza vera >= 3.0 m. Pattern principali: LEFT+REAR 1846, FRONT+LEFT 736,
  FRONT+RIGHT 611, REAR+RIGHT 513 (proprio le coppie di vicini a 90°).
- d_min onboard in formazione nominale, sensori CLEAN, FORMATION_FOLLOW (3-25 s): quadrato 2.45-2.62 m per tutti i
  droni (pattern a due settori); `line6` 3.05 m (un solo settore). Il quadrato vive quindi **dentro la banda di
  isteresi** (d_warning 2.4 / d_warning_exit 2.7), a 0.05-0.2 m sopra d_warning. Il progetto assume il contrario
  (`config.py`: `queue_spacing` "> d_warning_exit + estimate margin: queued neighbours never trigger the warning";
  ordine d_nominal > d_warning_exit), cosa vera solo per i target a un settore.
- Con rumore LOW la distanza d'eco scende di pochi cm, d_lower va sotto 2.4 e scatta il warning; si esce solo a 2.7,
  cioè quando la fusione si rompe (salto di ~0.4 m, più dell'isteresi di 0.3 m): ciclo SW -> RECOVERY -> SW. In CLEAN
  `lost_drone_timeout` non ha nessun ingresso in SW.
- `lost_drone_timeout` (drone 2 perso a t = 8 s): dopo la perdita, l'angolo di drone 0 (REAR+RIGHT = 2 e 1) e quello di
  drone 3 (FRONT+LEFT = 1 e 2) scompaiono; resta solo quello di **drone 1** (LEFT+REAR = 0 e 3, entrambi sani), che è
  anche il più basso in nominale (2.50 m). In tutti i 24 run non-PASS drone 1 passa 0-47 s su 132 in formazione, il
  resto in SW/RECOVERY; le RECOVERY di drone 1 nascono quasi tutte da `separation_clear` e tornano in SW.
- Il timer di vacanza avanza solo in FORMATION_FOLLOW / WAIT_REJOIN / DEGRADED (`perception.track_missing`,
  `IN_FORMATION`): drone 1 non raggiunge mai t_rejoin = 30 s e non dichiara la vacanza in nessuno dei 24 run (in un
  caso la dichiara a 93 s e la perde a 121 s). Il requisito "P3-deg vacancy" (ogni drone sano dichiara lo slot
  vacante) fallisce in 24/24.
- Le RECOVERY nei quadrati (`formation_recovery_head_current`, `lost_drone_rejoin`) e nel tee
  (`p1_vertical_escape`, drone 1 nell'angolo LEFT+REAR) seguono lo stesso schema: in tutti i run quasi tutte le entrate
  in RECOVERY vengono da `separation_clear`, mentre `own_slot_lost` (errore di navigazione vero) è raro (1-26 per
  scenario).

**Run interessati.** Causa primaria nei 24 `lost_drone_timeout` (in 22 da sola, in 2 insieme a lunghe RECOVERY sotto
deriva MEDIUM: B0 MEDIUM-ALL s1, B2_BLACKOUT MEDIUM-ALL s1); concausa nei 12 run quadrato che falliscono "P3
recovered" e nei 3 run del tee. B0 = B2: il fenomeno è nel livello sonar, che B2 non modifica.

**Tipo.** Non è un errore di codice: il bound è sound (verifica formale S0). È un'incoerenza di progetto fra il bound
(B) applicato ai target fusi, i template con vicini a 90° e le soglie di warning: la geometria nominale del quadrato
produce una distanza onboard inferiore a d_warning_exit.

### C2 - Deriva di navigazione lungo-traccia non corretta (MEDIUM)

**Evidenze.** Errore vero del drone peggiore nella seconda metà del run, componente lungo-traccia / laterale:
- `formation_six` (line6, vicini solo laterali): run B0 falliti 1.1-1.7 m / 0.1-0.6 m; B2 col canale (MEDIUM, 9/9
  PASS) 0.45-0.83 m; B2_BLACKOUT come B0 (1/3 fallito). In CLEAN 0.03-0.06 m.
- Quadrati: run falliti B0 / B2_BLACKOUT / B2_MEDIUM MEDIUM-ONE 1.1-2.4 m lungo-traccia; B2 MEDIUM-ALL col canale
  0.35-0.80 m (PASS 6/6).

**Meccanismo.** In una linea abreast l'eco laterale misura la spaziatura laterale ma è quasi insensibile allo
scostamento lungo-traccia: la deriva relativa lungo la direzione di marcia (DVL rumoroso/outage, bias di heading del
profilo MEDIUM) non è osservabile dal sonar di B0. Nei quadrati i vicini avanti/dietro la renderebbero osservabile, ma
C1 tiene i droni fuori da FORMATION_FOLLOW. Le fix acustiche (range-bearing) rendono osservabile il residuo
lungo-traccia: è la funzione prevista di B2.

**Run interessati.** Primaria nei 6 `formation_six` (B0 5, B2_BLACKOUT 1); concausa nei 12 quadrati, nei 3 tee e
nei 3 casi gate (C5).

**Tipo.** Limite fisico/di osservabilità di B0, non un bug.

### C3 - Ambiguità di associazione eco-slot (sonar a fascio largo, solo distanza)

**Evidenze.**
- 11 ri-occupazioni false dello slot perso in `lost_drone_timeout` (B0 6, B2 5): drone 2 fermo 18-28 m indietro,
  `SLOT_REOCCUPIED` provocato da un drone sano che si trova alla distanza giusta in un settore compatibile (il drone
  vero più vicino al punto dello slot è a ~3 m, quindi non è "nello slot"). La vacanza viene revocata e i vicini
  tornano ad aspettare. In tutto 12 run non-PASS hanno almeno una revoca.
- 9 false vacanze secondo il referee (vicino sano e vicino al suo slot dichiarato vacante) in 8 run: quadrati
  MEDIUM, `lost_drone_timeout` e `gate_single` B2_BLACKOUT s2.
- L'associazione di `check_formation` è già esclusiva (greedy per residuo, un eco per slot): un vicino spostato con
  residuo minore verso lo slot vacante "prende" quello slot e il suo diventa non visto (falsi missing tardivi,
  "still waiting").
- In B2 la regola A-D blocca la conferma solo con fix fresche del drone perso; a 18-28 m (fuori portata/ombra) non ce
  ne sono e B2 torna al comportamento di B0.

**Tipo.** Limite del sensore (cono di 120°, nessun bearing), aggravato da C1 che sposta i droni. Le decisioni di
vacanza/reinclusione sono di liveness, non di sicurezza.

### C4 - Echi fantasma vicini (rumore d'intensità MEDIUM)

**Evidenze.** Target ostacolo a r < 1.2 m: CLEAN 0, LOW 1 passo in 48 run, MEDIUM-ALL 3.3 episodi/run, MEDIUM-SUBSET
2.1, MEDIUM-ONE 0.7. Durano quasi sempre un passo (0.1 s), sono distribuiti uniformemente sui sei settori e compaiono
con la distanza vera minima >= 2.77 m. Su 1045 ingressi in COLLISION_AVOIDANCE, 2 con distanza vera < 2.4 m e 0
sotto 1.7 m.

**Meccanismo.** Il rumore additivo Rayleigh del profilo MEDIUM (sigma 0.075, soglia 0.30) produce superamenti di
soglia; due superamenti entro 0.5 m in 2 delle ultime 3 catture passano la conferma M-of-N. La stima analitica (circa
1.2e-3 fantasmi confermati per cattura e sensore, ~12% sotto 1.2 m) concorda con quanto osservato. La finestra di 0.5 m
non è larga: è quanto serve a un eco vero col rumore di distanza MEDIUM (sigma 0.10 m: 3 sigma sqrt(2) + moto ~0.52 m).

**Tipo.** Comportamento conservativo corretto dato il rumore dichiarato; costo di liveness moderato (CA/SW brevi
seguite da RECOVERY). Ridurlo richiede una conferma più lunga (es. 3 di 4), cioè più latenza nel budget di staleness
della prova P1: non è un cambiamento da fare ora.

### C5 - Gate: deriva → barra non spiegata → ostacolo; aiding attivo solo entro 2.0 m

**Evidenze.**
- `integrated_short` MEDIUM-ONE s0, B0 e B2 (stessa dinamica): il drone degradato (errore di navigazione 0.72-1.1 m,
  gli altri < 0.2 m) è in MUTEX_PASS dal t = 62 s ma resta davanti a G06 (s ~ -3.3 m) fino alla deadline, oscillando
  fra SW/CA e MUTEX_PASS. In FRONT ha per ~1400 passi un eco UNKNOWN a 2.42-2.72 m, mentre le barre compaiono come
  STRUCTURE solo a 4-5.5 m. `loc_ok` resta falso per tutto il run.
- `perception/structure_aiding.py` attiva la correzione sulla mappa solo entro `AID_RANGE_M` = 2.0 m da una barra,
  mentre SEPARATION_WARNING scatta a 2.4 m ed esce a 2.7 m: il drone oscilla proprio in 2.4-2.7 m dalla barra "non
  spiegata" e non arriva mai sotto 2.0 m. L'aiding non parte, la localizzazione resta non valida e l'hold di
  rilocalizzazione (`control/flows.py`) non lo lascia entrare nella regione critica: livelock sicuro.
- Variante in coda, `gate_single` B2_BLACKOUT MEDIUM-ALL s2: dopo il passaggio di drone 0 (30.6 s) i droni 1 e 2
  restano in MUTEX_YIELD fino alla deadline col latch BUSY. Dal t ~ 60 s drone 2 (in coda a s = -5.2 m) vede in FRONT
  a 4.6-4.8 m (distanza delle barre) un eco DYNAMIC/UNKNOWN che cade nella finestra del corridoio: aggiorna
  `last_corridor_t`, quindi il timeout `t_occ_max` (25 s dall'ultimo eco di corridoio) non scatta mai. Prima (25-40 s)
  drone 2 non ha visto drone 0 davanti e non ha ottenuto il testimone d'uscita; nel B0 dello stesso seed lo ottiene a
  26.2 s (FRONT 6.32 m) e passa. Il perché della mancata osservazione a 25-40 s non è ancora chiarito.

**Tipo.** Difetto strutturale di progetto (raggio di attivazione dell'aiding incoerente con le soglie di separazione)
per il primo caso; per la variante in coda il meccanismo (eco di barra non spiegata che tiene il latch) è confermato,
la catena completa no. B0 = B2 (con canale silenzioso B2 coincide con B0 nel latch).

### C6 - Classificazione

I 2 LIVENESS_FAILURE (`lost_drone_timeout` LOW s0, B0 e B2) hanno la stessa causa dei 22 SAFE_DEGRADED dello stesso
scenario: l'etichetta cambia solo perché in LOW non ci sono FAILSAFE da guasto sensore né false vacanze. I criteri
non vanno cambiati dopo aver visto i risultati; si può solo aggiungere, nei report, un tag di causa che non tocca la
classe.

## 3. I 2 LIVENESS_FAILURE (lost_drone_timeout, LOW, seed 0)

- **B0.** Drone 0 dichiara lo slot 2 vacante a 42.7 s, drone 3 a 77.6 s (in ritardo per 24 s di RECOVERY e 2.5 s di
  SW); drone 1 mai (3.6 s in formazione, 51.5 s in SW, 77 s in RECOVERY dopo il guasto). A 137.3 e 137.8 s le vacanze
  di 3 e 0 vengono revocate da echi di droni sani (C3: drone 0 visto a 2.99 m dal punto dello slot dal drone 3, e
  viceversa); alla deadline (140.15 s) nessuno ha lo slot vacante e drone 0 aspetta un vicino presente (missing [3]).
  Cause: C1 (primaria) + C3.
- **B2.** Rilevazione della perdita in ~2 s (regola A-D); vacanze di 0 e 3 a 45.6 e 53.4 s, stabili (nessuna revoca);
  drone 1 mai (6.4 s in formazione, 69.5 s in SW, 56.2 s in RECOVERY). Causa: solo C1.

## 4. SAFE_DEGRADED per scenario, profilo e protocollo

| Scenario | Run non-PASS | Profili (protocollo: seed) | Requisiti mancanti | Causa primaria / concause |
|---|---|---|---|---|
| lost_drone_timeout | 24 (22 SD + 2 LF) | B0 LOW s0,s2; B0 M-ONE s0-2; B0 M-SUBSET s0-2; B0 M-ALL s0-2; B2_LOW s0-2; B2_MEDIUM M-ONE s2; B2_MEDIUM M-SUBSET s0-2; B2_MEDIUM M-ALL s0-2; B2_BLACKOUT M-ALL s0-2 | P3-deg vacancy (24/24), a volte re-formed, still waiting | C1 / C3, C2 (MEDIUM), C4 |
| formation_recovery_head_current | 7 | B0 M-ALL s0,s1; B0 M-SUBSET s0,s1; B2_MEDIUM M-ONE s2; B2_BLACKOUT M-ALL s1,s2 | P3 recovered | C1 + C2 / C3, C4 |
| lost_drone_rejoin | 5 | B0 M-ALL s0-2; B0 M-SUBSET s0; B2_MEDIUM M-ONE s2 | P3 recovered | C1 + C2 / C3, C4 |
| formation_six | 6 | B0 M-ALL s0; B0 M-ONE s0; B0 M-SUBSET s0-2; B2_BLACKOUT M-ALL s0 | P3 recovered | C2 / C4 |
| p1_vertical_escape | 3 | B0 M-ALL s2; B2_BLACKOUT M-ALL s0,s2 | P3 recovered | C1 (drone 1 nell'angolo del tee) + C2 / C3, C4 |
| integrated_short | 2 | B0 M-ONE s0; B2_MEDIUM M-ONE s0 | progress, gate, P3 recovered | C5 |
| gate_single | 1 | B2_BLACKOUT M-ALL s2 | progress, gate, P3 recovered | C5 (variante in coda) |

## 5. Differenze B0 / B2

Confronto su celle accoppiate (stesso profilo/scenario/seed, entrambi validi):

| Sensori | Variante B2 | Coppie | non-PASS B0 | non-PASS B2 |
|---|---|---|---|---|
| CLEAN | B2_CLEAN / canale MEDIUM / BLACKOUT | 33 / 22 / 21 | 0 | 0 |
| LOW-ALL | B2_LOW | 24 | 2 | 3 |
| MEDIUM-ONE | B2_MEDIUM | 24 | 5 | 4 |
| MEDIUM-SUBSET | B2_MEDIUM | 24 | 9 | 3 |
| MEDIUM-ALL | B2_MEDIUM | 23 | 10 | 3 |
| MEDIUM-ALL | B2_BLACKOUT | 21 | 10 | 9 |

- B2 col canale riduce i fallimenti dove agisce l'informazione acustica: deriva relativa (C2), falsi missing,
  rilevazione della perdita (~2 s in B2 col canale; in B0 da meno di 1 s fino a 112 s, o mai).
- C1, C4 e C5 sono nel livello sonar/navigazione assoluta e sono identici in B0 e B2: i non-PASS residui di B2 col
  canale sono tutti `lost_drone_timeout` (C1) più i casi MEDIUM-ONE di quadrati e gate.
- LOW: 3 contro 2, tutti `lost_drone_timeout` (C1): differenza nel rumore statistico (realizzazioni indipendenti).
- B2_BLACKOUT si comporta come B0 (10 contro 9), come previsto dal progetto (canale silenzioso = B0); il caso
  `gate_single` s2 è C5 e non mostra un difetto specifico di B2.

## 6. Problemi realmente correggibili e proposta generale

In ordine di impatto (un problema alla volta, ciascuno verificato con pochi test mirati prima del successivo):

1. **C1 - bound dei target multi-settore** (componente: `holo_fleet/perception/sonar_geometry.py`,
   `target_distance_lower` / `centre_in_cone`; nessuna soglia, automa o template modificato).
   Proposta: per un target visto da due settori adiacenti i cui coni si sovrappongono, il centro sta nel cono di
   almeno uno dei due membri; il bound può essere il **minimo dei bound (A) dei membri** invece del minimo dei bound
   (B), ma solo in un **dominio di validità esplicito** (lontano dal near-field, dove l'estensione angolare dello scafo
   supera la sovrapposizione), da derivare geometricamente; fuori dal dominio resta il bound attuale. Non è
   l'assunzione "centro in ogni cono" (mutazione S0m, unsound): è un'unione di casi, ciascuno sound.
   Evidenza preliminare (Monte Carlo offline sulla geometria di S0, nessuna modifica al codice):
   - nessuna sovrastima su 40000 pose con uno scafo e 40000 con due scafi fusi (1-6 m) e sulle direzioni d'angolo
     (1-3 m); margine minimo +0.044 / +0.056 / +0.057 m;
   - sovrastima fino a 0.084 m nel near-field (centro a 0.8-0.83 m): da qui il dominio di validità;
   - stringe di ~0.5 m: caso dei log LEFT 2.92 / REAR 3.07 da 2.55 a 3.04 m, sopra d_warning_exit.
   Effetto atteso: ripristina d_nominal(onboard) > d_warning_exit per i template con vicini a 90°, elimina la maggior
   parte dei 4203 warning spuri e il ciclo SW↔RECOVERY che blocca il timer di vacanza.
   Alternativa scartata: dividere il target usando le fix di B2 (introdurrebbe un'assunzione su oggetti non
   cooperativi e varrebbe solo per B2).

2. **C5 - raggio di attivazione del structure aiding** (componente: `holo_fleet/perception/structure_aiding.py`;
   il livello di sicurezza resta invariato: gli echi restano ostacoli per CA/SW e `structure_tol_m` non si allarga).
   Proposta: rendere coerente l'attivazione del fit sulla mappa con le soglie di separazione, cioè permettere fit e
   conferma con le barre visibili fino ad almeno d_warning_exit + margine (oggi 2.0 m < d_warning 2.4 m), mantenendo
   tutti i criteri di accettazione esistenti (>= 2 settori, unicità, consistenza, clearance). Prima di proporre una
   modifica per la variante in coda (eco di barra che tiene il latch BUSY) serve chiarire la catena completa di
   `gate_single` B2_BLACKOUT s2; non va toccato il latch prima di allora (P2).

3. **C3 - reinclusione in B2** (componente: `holo_fleet/perception/formation_perception.py`, solo ramo B2; decisioni
   di liveness, la sicurezza non è coinvolta).
   Proposta: associazione in due passi, prima gli echi spiegati da fix fresche di vicini identificati (riservati a
   quell'identità), poi i candidati di soli range per gli slot senza fix; un eco già spiegato dalla fix di un altro
   vicino non può confermare uno slot vacante. In B0 non c'è informazione per disambiguare: limite da documentare.

Non correggibili ora o non bug: C2 (limite di osservabilità di B0, già affrontato da B2), C4 (limite del rumore
dichiarato; una conferma più lunga tocca la prova P1), C6 (solo tag diagnostico, i criteri non cambiano).

## 7. Test mirati per ciascuna correzione

**C1**
- Estendere `formal/check_separation.py` S0: pattern a due settori adiacenti con uno scafo, con due scafi fusi, insieme
  near-field (0.6-1.5 m) e direzioni d'angolo; deve valere d_hat <= d su tutti i campioni, e S0m deve continuare a
  rilevare la mutazione unsound.
- Test unitario: echi LEFT 2.92 / REAR 3.07 (geometria del quadrato) -> bound >= d_warning_exit; un caso near-field
  -> bound invariato.
- Suite completa e verifiche formali (S0-S1, M-checks) invariate.
- 4-6 run HoloOcean mirati: `lost_drone_timeout` LOW s0 B0 e B2 (i due LIVENESS) e
  `formation_recovery_head_current` LOW/MEDIUM-ALL s0. Criteri: nessun ingresso in SW con distanza vera >= 3 m da target
  multi-settore, drone 1 dichiara la vacanza, P1/P2 invariati.

**C5**
- Test unitario/replay su `structure_aiding` con deriva laterale iniettata di 0.8-1.0 m e drone a 2.4-2.7 m dalla
  barra: il fit converge e conferma la localizzazione; con un veicolo davanti alla barra l'eco resta ostacolo
  (`tests/test_structure_aiding.py` esistente deve restare verde).
- Verifiche formali del gate (P2, M-checks) invariate.
- 2 run: `integrated_short` MEDIUM-ONE s0 B0 e B2.

**C3**
- Test unitario: slot vacante con l'eco di un vicino spostato alla distanza dello slot, e fix fresca di quel vicino
  -> nessun `SLOT_REOCCUPIED` in B2; senza fix, comportamento B0 invariato.
- 2-3 run: `lost_drone_timeout` MEDIUM-SUBSET s1-s2 B2.

Solo dopo questi passi: rilanciare i 108 run mancanti della campagna (con le celle mutex rimaste INVALID) e rifare,
a codice corretto, i run toccati dalle correzioni.

## 8. Limiti di questa diagnosi

- Con 3 seed e realizzazioni indipendenti, le differenze B0/B2 per cella sono indicative.
- La catena completa del mancato testimone d'uscita in `gate_single` B2_BLACKOUT s2 (25-40 s) non è ricostruita.
- La verifica del bound proposto per C1 è un Monte Carlo offline sulla geometria del modello: non sostituisce
  l'estensione della verifica S0 né i run mirati.

## Riproduzione

Script in `scripts/` (Python dell'ambiente `holo_fleet_ha`, dalla radice del repository):
- `sw_scan.py <raw> <out.json>`: ingressi in SW contro la verità, tempi di modo, vacanze;
- `nonpass.py <raw> <out.json>`: per drone, tempi di modo dopo il guasto e cause di transizione;
- `phantom.py <raw> <out.json>`: target vicini (< 1.2 m) per profilo;
- `along_lat.py <scenari...>`: decomposizione dell'errore vero di formazione;
- `reocc.py`: chi ri-occupa lo slot;
- `tags.py <dir>`: tag di causa;
- `union_bound_mc.py`, `union_corner.py`: verifica Monte Carlo del bound proposto.

Dati in `data/`.
