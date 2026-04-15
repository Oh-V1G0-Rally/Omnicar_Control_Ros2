# AGENTS.md

## Scopo di questo repository

Questo repository contiene il progetto `OMNICAR`, una piattaforma mobile
omnidirezionale a 4 ruote usata come base sperimentale per una tesi focalizzata
sul controllo per autonavigazione.

Il repository include:

- firmware embedded su ESP32 in `Omnicar/`
- stack ROS2 principale in `omnicar_ws/`
- stack legacy ROS1 in `omnicar_ros_ws/`
- repository storici e riferimenti progettuali, tra cui `5dpo_ratf_2023-main/`

Lo scopo attuale non e' piu' completare una migrazione feature-by-feature del
vecchio stack competition, ma costruire una piattaforma ROS2 pulita,
riproducibile e adatta a sviluppare, testare e confrontare strategie di
controllo per navigazione autonoma.

## Obiettivo della tesi

L'obiettivo principale del progetto e' sviluppare e validare una pipeline di
autonavigazione centrata sul controllo del moto del robot omnidirezionale.

La progressione tecnica desiderata e':

1. raggiungimento di waypoint singoli
2. tracking di traiettorie semplici
3. tracking di traiettorie complesse
4. introduzione di un controllo avanzato di tipo MPCC
5. sviluppo di uno o piu' controlli non lineari
6. confronto sperimentale tra le diverse strategie

Il progetto deve quindi diventare una piattaforma di ricerca sul controllo,
non una replica del comportamento gara Robot@Factory.

## Cosa NON e' obiettivo del progetto

Non sono obiettivi prioritari del repository, salvo richiesta esplicita:

- logiche competition per movimentazione box
- gestione del solenoide come task applicativo principale
- workflow di gara con ordine box, mission logic o server UDP dedicati
- porting integrale delle logiche di `5dpo_ratf_2023-main`
- integrazione di componenti accessori non necessari al controllo di traiettoria

Questo significa che moduli relativi a box picking, sequenze di gara o
interfacce costruite per Robot@Factory vanno trattati come riferimenti storici,
non come baseline architetturale.

## Struttura del repository

### `Omnicar/`

Firmware ESP32 basato su Arduino/PlatformIO.

Responsabilita' principali:

- lettura encoder
- applicazione del controllo basso livello ruota
- attuazione PWM
- scambio seriale con il PC

File chiave:

- `platformio.ini`
- `src/main.cpp`
- `src/Robot.cpp`
- `src/CtrlPID.cpp`
- `src/MRAC.cpp`
- `include/Robot.h`
- `include/MotorAutotuner.h`
- `lib/channels/`

Nota:

- il firmware e' la sorgente di verita' per il protocollo seriale
- il controllo basso livello ruota e il controllo alto livello di navigazione
  vanno tenuti concettualmente separati

### `omnicar_ws/`

Workspace ROS2 principale e destinazione finale dello sviluppo.

Questo workspace deve contenere lo stack realmente usato per:

- comunicazione con il robot
- odometria e stima posa
- localizzazione minima necessaria
- nodi di controllo per waypoint e traiettorie
- strumenti di test, tuning, logging e confronto controllori

Package attualmente rilevanti:

- `5dpo_ratf_driver-main`
  - driver ROS2 piu' coerente con il firmware e il protocollo seriale attuale
- `sdpo_drivers_interfaces`
  - messaggi e servizi ROS2 di base
- `5dpo_serial_port-main`
  - libreria seriale
- `serial_communication_channels-main`
  - parser protocollo lato PC
- `sdpo_driver_omnijoy`
  - teleop/joystick utile per test manuali
- `sdpo_ros_odom`
  - odometria
- `sdpo_ratf_ros_localization`
  - localizzazione disponibile oggi
- `ldlidar_stl_ros`
  - lidar

### `omnicar_ros_ws/`

Workspace legacy ROS1.

Va usato come riferimento storico e funzionale, non come target da
replicare integralmente.

Serve soprattutto per:

- ricostruire topic e interfacce originarie
- verificare ipotesi sulla pipeline di odometria/localizzazione
- recuperare implementazioni o parametri utili

### `5dpo_ratf_2023-main/`

Repository di riferimento storico/architetturale.

Contiene:

- una versione competition-oriented dello stack ROS1
- path planning e mission logic di gara
- moduli opzionali IMU, camera, HMI
- configurazioni launch complete per Robot@Factory

Va usato con cautela:

- utile come archivio di idee, struttura e algoritmi
- non va preso come baseline da copiare in blocco
- la presenza di logiche box/solenoide non implica che debbano entrare nello
  stack finale della tesi

## Obiettivo tecnico corrente

L'obiettivo attuale del repository e' costruire un'infrastruttura ROS2 stabile
per sperimentare controllori di autonavigazione sull'OMNICAR.

La priorita' non e' aggiungere nuove feature applicative, ma chiudere un loop
di controllo riproducibile:

- stima della posa del robot
- riferimento di posa o traiettoria
- controllore
- comando velocita'
- misura delle prestazioni

In termini pratici, il primo deliverable utile e':

- raggiungere waypoint assegnati in piano con un controllore semplice e
  parametrizzabile

## Architettura concettuale desiderata

La piattaforma va pensata a strati.

### Strato 1: Hardware e comunicazione

Include:

- firmware ESP32
- protocollo seriale
- driver ROS2

Responsabilita':

- scambio affidabile ROS2 <-> robot
- lettura encoder
- invio riferimenti di velocita' ruota o comandi equivalenti

### Strato 2: State estimation

Include:

- odometria
- TF
- eventuale localizzazione
- eventuale fusione sensori

Responsabilita':

- fornire una posa consistente del robot
- mantenere frame e topic stabili
- consentire al controllore di lavorare su uno stato affidabile

### Strato 3: Motion control

Include:

- controllore go-to-point
- controllore di tracking traiettoria
- controllori avanzati

Responsabilita':

- ricevere un riferimento di posa o traiettoria
- calcolare `cmd_vel`
- rispettare limiti cinematici e di sicurezza

### Strato 4: Sperimentazione e benchmarking

Include:

- generatori di traiettoria
- nodi di logging
- valutazione metriche
- script per confronto offline

Responsabilita':

- misurare prestazioni
- rendere ripetibili gli esperimenti
- supportare il confronto tra strategie di controllo

## Modello di riferimento e convenzioni

Per tutto cio' che riguarda il controllo alto livello, il robot va trattato
prima di tutto come piattaforma omnidirezionale planare.

Stato minimo di interesse:

- `x`
- `y`
- `yaw`
- velocita' nel frame robot

Ingressi di controllo di alto livello:

- `vx`
- `vy`
- `w`

Convenzioni desiderate:

- frame globale di riferimento stabile, tipicamente `odom` o `map`
- frame body del robot coerente con `base_footprint` o equivalente
- definizione esplicita del verso positivo di `vx`, `vy`, `w`
- uso consistente di radianti, metri e secondi

Nota importante:

- il layer di controllo alto livello deve essere espresso in termini di moto
  planare del robot
- la conversione da `cmd_vel` a riferimenti ruota deve rimanere confinata ai
  layer sottostanti o a moduli chiaramente separati

## Interfacce ROS2 raccomandate per il controllo

I nuovi nodi di controllo devono usare interfacce semplici, stabili e facili da
loggare.

Input raccomandati:

- posa del robot da `nav_msgs/Odometry`, `geometry_msgs/PoseStamped` o TF
- waypoint target da topic dedicato
- traiettoria di riferimento da topic o da generatore interno

Output raccomandato:

- `geometry_msgs/Twist` su `cmd_vel`

Interfacce da favorire:

- riferimento waypoint singolo
- lista waypoint
- traiettoria campionata o parametrica
- topic di stato del controllore
- topic di errore di tracking

Quando possibile, i controllori devono pubblicare anche:

- errore posizione
- errore heading
- riferimento corrente
- stato interno minimo utile al debug

## Parametri di controllo da rendere espliciti

Ogni controllore introdotto nel repository deve esporre in modo chiaro almeno:

- frequenza di controllo
- guadagni
- limiti su `vx`, `vy`, `w`
- limiti su accelerazione o rate se applicati
- tolleranze di arrivo
- scelta del frame di controllo
- abilitazione/disabilitazione dei termini integrali o derivativi

E' preferibile evitare:

- guadagni hardcoded nel codice
- parametri impliciti non documentati
- cambiamenti di segno o convenzioni nascosti

## Direzione scientifica e roadmap

### Fase 1: Waypoint control

Primo obiettivo pratico:

- assegnare un punto target `(x, y, yaw)`
- far convergere il robot al target
- ottenere un comportamento ripetibile

Controllore iniziale raccomandato:

- PID semplice o anche P/PID cinematico in body frame

Input minimi:

- posa del robot
- waypoint target

Output:

- `cmd_vel`

Temi da chiarire in questa fase:

- scelta del frame di controllo
- saturazione velocita'
- soglie di arrivo
- gestione orientamento finale
- robustezza rispetto a errori odometrici

Deliverable:

- nodo ROS2 `go_to_point_controller` stabile e parametrizzabile

### Fase 2: Trajectory tracking

Dopo aver chiuso bene il reach-a-point, si passa al tracking continuo.

Traiettorie di interesse:

- retta
- cerchio
- lemniscata / figura a otto
- traiettorie spline

Obiettivo:

- seguire una traiettoria tempo-parametrizzata o path-parametrizzata

Aspetti rilevanti:

- tracking error in frame robot
- feedforward + feedback
- gestione velocita' di riferimento
- continuita' di curvatura

Deliverable:

- nodo ROS2 di trajectory tracking
- almeno una traiettoria benchmark robusta, preferibilmente la figura a otto

### Fase 2.5: Infrastruttura sperimentale

Prima di introdurre controllori piu' avanzati, e' necessario rendere
riproducibile il testing.

Elementi desiderati:

- launch dedicati per esperimenti
- registrazione rosbag dei topic essenziali
- export CSV o script di post-processing
- naming coerente degli esperimenti
- configurazioni salvate dei guadagni

Deliverable:

- pipeline minima per eseguire, registrare e confrontare test ripetuti

### Fase 3: MPCC

Una volta stabilizzati modello, riferimenti e tracking base, introdurre un
controllore avanzato di tipo MPCC.

Obiettivi:

- minimizzare contouring error e lag error
- gestire vincoli su velocita' e input
- aumentare prestazioni sul tracking di traiettorie

Prerequisiti:

- modello cinematico/dinamico ben definito
- traiettoria parametrica affidabile
- metriche di confronto gia' definite

Deliverable:

- implementazione ROS2 di MPCC o integrazione con solver adatto
- benchmark comparativo con controllore PID/classico

### Fase 4: Controllo non lineare

Obiettivo:

- esplorare controllori non lineari per il tracking della piattaforma
  omnidirezionale

Possibili direzioni:

- feedback linearization
- Lyapunov-based tracking
- backstepping
- sliding mode, se giustificato

Il punto non e' aggiungere complessita' gratuitamente, ma produrre una
comparazione scientificamente difendibile rispetto a:

- semplici controllori cinematici
- MPCC

Deliverable:

- almeno una strategia non lineare implementata e testata

### Fase 5: Confronto sperimentale

Questa fase e' parte integrante della tesi, non un accessorio.

Metriche consigliate:

- errore finale sul punto
- tempo di assestamento
- overshoot
- errore RMS di tracking
- errore massimo laterale
- errore di heading
- effort di controllo
- robustezza a disturbi e rumore

Scenari minimi:

- waypoint singolo
- sequenza di waypoint
- cerchio
- figura a otto

Aspetti da confrontare in modo coerente:

- accuratezza
- rapidita'
- regolarita' del comando
- robustezza a variazioni iniziali
- robustezza a rumore o drift
- facilita' di tuning

## Criteri per introdurre un nuovo controllore

Un nuovo controllore non dovrebbe essere aggiunto al repository se non sono
chiari:

- modello di riferimento usato
- input richiesti
- output prodotti
- ipotesi di validita'
- parametri da tarare
- metriche con cui verra' confrontato

Ogni nuovo controllore dovrebbe arrivare con:

- launch minimo
- file parametri
- descrizione sintetica della legge di controllo
- scenario di test raccomandato

Questo vale in particolare per:

- MPCC
- controllori non lineari
- varianti del PID o del trajectory tracker

## Stato del progetto da assumere

Va assunto che:

- `omnicar_ws/` e' gia' il workspace principale
- molte funzionalita' legacy sono gia' state riallineate o portate
- non serve continuare a ragionare in termini di semplice migrazione ROS1->ROS2
- la parte ad alto valore ora e' lo sviluppo dei nodi di controllo e del setup
  sperimentale

## Driver ROS2 da considerare baseline

Il driver di riferimento rimane:

- `omnicar_ws/src/5dpo_ratf_driver-main`

Motivi:

- e' il package piu' coerente con il firmware ESP32 e il protocollo seriale
- costituisce la base piu' solida per chiudere il loop hardware-in-the-loop

Il package `omnicar_ws/src/omnicar_driver` va trattato come secondario o
prototipale, salvo futura riallineazione esplicita.

## Protocollo seriale

Il protocollo seriale tra ROS2 e firmware e' un contratto di sistema.

Direzione ESP32 -> PC:

- `g`, `h`, `i`, `j`
  - tick encoder dei 4 motori
- `k`
  - sample time
- `s`, `t`
  - stati switch
- `r`
  - reset

Direzione PC -> ESP32:

- `G`, `H`, `I`, `J`
  - riferimenti di velocita' angolare delle ruote
- `K`
  - PWM diretto
- `L`, `M`
  - comandi ausiliari legacy

Nota operativa:

- i canali `L`, `M` e la logica solenoide non sono obiettivi della tesi
- non vanno rimossi se esistenti, ma non devono guidare l'architettura futura

## Decisioni architetturali da mantenere

- una sola implementazione del driver robot deve rimanere centrale
- il protocollo seriale va preservato salvo rifattorizzazione esplicita del
  firmware
- i topic e i frame usati dal controllo devono essere stabili e ben documentati
- il controllo alto livello deve lavorare su `cmd_vel` o interfacce equivalenti
  ben definite
- le logiche di gara non devono infiltrarsi nei package core di controllo
- ogni controllore deve essere sviluppato in modo modulare e intercambiabile

## Package da sviluppare o consolidare

I package futuri dovrebbero essere organizzati in modo pulito. Esempi
ragionevoli:

- un package dedicato al controllo, ad esempio `sdpo_motion_control`
- un package per traiettorie/generatori di riferimento
- un package o cartella per logging e benchmarking

Funzionalita' desiderate nel layer di controllo:

- `go_to_point`
- sequenza waypoint
- trajectory tracking
- switch semplice tra controllori
- parametri ROS2 chiari e centralizzati
- log di errori e comandi

Struttura minima consigliata:

- nodo controllore
- file parametri
- launch file di test
- messaggi o topic di debug solo se realmente utili

## Simulazione e test

Se viene introdotta una simulazione, questa deve essere funzionale alla tesi e
non diventare un progetto parallelo.

Ordine di preferenza:

1. test su dati registrati / replay
2. test hardware con velocita' limitate e scenario controllato
3. eventuale simulazione semplificata

La simulazione e' utile se:

- accelera il tuning iniziale
- consente confronti ripetibili tra controllori
- usa lo stesso schema di interfacce ROS2 del robot reale

Non e' utile se:

- introduce una seconda architettura non coerente con l'hardware reale
- richiede molto lavoro infrastrutturale senza aiutare il confronto controllori

## Cose da evitare

- riprendere dal repository 2023 logiche di box handling come se fossero parte
  del core del progetto
- introdurre dipendenze competition-specifiche come UDP mission server se non
  strettamente necessarie alla tesi
- mischiare controllo basso livello ruota e controllo alto livello traiettoria
- cambiare contemporaneamente firmware, driver e controllore senza isolare i
  problemi
- mantenere piu' pipeline concorrenti per la stessa funzione
- valutare controllori diversi senza uno stesso set di metriche e scenari

## Uso dei repository storici

### `omnicar_ros_ws/`

Usarlo per:

- verificare topic, frame e parametri storici
- recuperare implementazioni legacy quando servono davvero

Non usarlo per:

- imporre una migrazione totale come obiettivo

### `5dpo_ratf_2023-main/`

Usarlo per:

- recuperare idee di planner, traiettorie, struttura launch o benchmark
- studiare moduli avanzati gia' esplorati in passato

Non usarlo per:

- reintrodurre box logic, mission logic e solenoide nella baseline della tesi
- assumere che tutti i suoi package siano maturi o coerenti con l'attuale
  hardware

## Prossimi passi raccomandati

Ordine consigliato:

1. verificare e fissare la pipeline minima `driver -> odom/localizzazione ->
   pose -> cmd_vel`
2. definire l'interfaccia del controllore waypoint
3. implementare un controllore PID semplice per `go_to_point`
4. validarlo su target statici
5. estenderlo a sequenze di waypoint
6. introdurre generatori di traiettoria
7. testare una traiettoria a figura di otto
8. solo dopo passare a MPCC
9. successivamente sviluppare controllo non lineare
10. costruire benchmarking e confronto formale

## Priorita' pratiche immediate

Nel breve termine, ogni attivita' dovrebbe aiutare almeno uno di questi tre
punti:

- ottenere una posa affidabile del robot in ROS2
- comandare il robot in velocita' planare in modo controllato
- misurare l'errore tra riferimento e moto reale

Se una modifica non contribuisce chiaramente a uno di questi tre punti, va
considerata secondaria rispetto agli obiettivi attuali della tesi.

## Nota per chi lavora su questo repository

Quando si apportano modifiche:

- privilegiare `omnicar_ws/` come destinazione finale
- usare `Omnicar/` come sorgente di verita' per protocollo seriale e controllo
  basso livello
- usare `omnicar_ros_ws/` e `5dpo_ratf_2023-main/` come riferimenti, non come
  obiettivi da copiare
- distinguere sempre tra infrastruttura robotica e sperimentazione di controllo
- ogni nuova implementazione deve aiutare il percorso della tesi:
  waypoint, traiettorie, MPCC, controllo non lineare, benchmarking
