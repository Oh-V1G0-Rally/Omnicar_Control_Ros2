# AGENTS.md

## Scopo di questo repository

Questo repository contiene il progetto `OMNICAR`, una piattaforma mobile omnidirezionale a 4 ruote con:

- firmware embedded su ESP32 in `Omnicar/`
- stack ROS2 in `omnicar_ws/`
- stack legacy ROS1/ibrido in `omnicar_ros_ws/`

L'obiettivo attuale del progetto e' completare la migrazione funzionale da `omnicar_ros_ws` a ROS2, consolidando tutto in `omnicar_ws` e rendendo operativo l'intero sistema end-to-end.

## Obiettivo corrente

Portare in ROS2 tutto cio' che era disponibile nel vecchio stack `omnicar_ros_ws`, inclusi:

- driver robot e comunicazione seriale con ESP32
- controllo da joystick
- odometria
- localizzazione
- lidar
- interfacce messaggi e servizi
- strumenti di tuning/calibrazione dove ancora utili

Il target non e' una semplice conversione sintattica dei package, ma uno stack ROS2 funzionante e coerente, con protocolli, topic, frame, launch e dipendenze allineati.

## Struttura del repository

### `Omnicar/`

Firmware ESP32 basato su Arduino/PlatformIO.

Contenuti chiave:

- `platformio.ini`
  - target `esp32dev`
  - seriale a `115200`
- `src/main.cpp`
  - setup del robot
  - inizializzazione seriale
  - loop di controllo
  - parsing dei comandi seriali
  - invio telemetria verso ROS/PC
- `src/Robot.cpp`
  - logica di alto livello del firmware
  - acquisizione encoder
  - selezione del controllore
  - invio dei canali seriali
- `src/CtrlPID.cpp`
  - controllore PID velocita' ruota
- `src/MRAC.cpp`
  - controllore adattativo alternativo
- `include/MotorAutotuner.h`
  - test/tuning motori
- `lib/channels/`
  - protocollo seriale a canali ASCII/esadecimale

### `omnicar_ws/`

Workspace ROS2 attivo. Questo deve diventare il workspace principale finale.

Package principali:

- `5dpo_ratf_driver-main`
  - driver ROS2 piu' completo attualmente presente
  - gestisce seriale, topic encoder, topic riferimenti motori e service solenoidi
- `omnicar_driver`
  - driver ROS2 alternativo/prototipale
  - attualmente non allineato al protocollo seriale del firmware
- `sdpo_drivers_interfaces`
  - messaggi e servizi ROS2 usati dal driver
- `5dpo_serial_port-main`
  - wrapper seriale asincrono
- `serial_communication_channels-main`
  - parser/protocollo seriale lato PC

### `omnicar_ros_ws/`

Workspace legacy ROS1 o di transizione. Va trattato come sorgente di verita' funzionale per capire cosa va portato in ROS2.

Package principali da migrare o riassorbire:

- `5dpo_ratf_ros_driver`
  - vecchio driver principale robot
- `5dpo_driver_omnijoy`
  - controllo robot da joystick
- `5dpo_ros_odom`
  - odometria
- `5dpo_ratf_ros_localization`
  - localizzazione
- `robot_pose_ekf`
  - fusione pose / EKF
- `ldlidar_stl_ros`
  - lidar e publish di scan
- `5dpo_ros_interfaces`
  - vecchie interfacce ROS1
- `5dpo_ros_serial_port`
  - vecchia libreria seriale
- `serial_communication_channels`
  - libreria protocollo seriale legacy

## Architettura funzionale attuale

### Firmware ESP32

Il firmware implementa il controllo basso livello dei motori.

Responsabilita':

- lettura encoder
- aggiornamento stato motori
- applicazione del controllore
- attuazione PWM
- comunicazione seriale con il PC/ROS

Controllori presenti nel firmware:

- PID velocita'
- MRAC

Stato attuale:

- il percorso attivo e' PID
- MRAC e' presente ma non e' il percorso selezionato di default
- non risultano implementazioni SMC/sliding mode nei sorgenti attuali

### Driver ROS2 attualmente piu' coerente

Il package da considerare come base principale e' `omnicar_ws/src/5dpo_ratf_driver-main`.

Motivi:

- usa il protocollo seriale a canali coerente con il firmware ESP32
- pubblica encoder e switch
- sottoscrive riferimenti motori
- gestisce watchdog e reconnessione seriale
- ha una struttura gia' vicina al vecchio `5dpo_ratf_ros_driver`

### Driver ROS2 secondario

`omnicar_ws/src/omnicar_driver` al momento sembra un tentativo parallelo. Usa un formato seriale diverso, di tipo testuale, e non coincide con il protocollo realmente usato dal firmware embedded.

Fino a prova contraria:

- non deve essere preso come base principale della migrazione
- va o riallineato completamente al firmware oppure dismesso per evitare duplicazioni

## Protocollo seriale

La comunicazione tra ROS2 e firmware e' basata su canali identificati da un carattere e da 32 bit codificati in esadecimale ASCII.

Direzione ESP32 -> PC:

- `g`, `h`, `i`, `j`
  - tick encoder dei 4 motori
- `k`
  - sample time / delta temporale
- `s`, `t`
  - stati switch
- `r`
  - reset

Direzione PC -> ESP32:

- `G`, `H`, `I`, `J`
  - riferimenti di velocita' angolare
- `K`
  - PWM diretto
- `L`, `M`
  - comandi ausiliari / solenoidi

Nota importante:

- ogni modifica lato ROS2 deve rimanere compatibile con questo protocollo, salvo decisione esplicita di rifattorizzazione completa anche del firmware

## Stato del progetto

### Cosa esiste gia'

- firmware ESP32 con controllo motori e seriale
- driver ROS2 `sdpo_ratf_driver`
- interfacce ROS2 base
- librerie seriali portate o riutilizzate

### Cosa manca per completare la migrazione

- joystick ROS2 equivalente al vecchio `5dpo_driver_omnijoy`
- odometria ROS2 integrata nello stack finale
- localizzazione ROS2 equivalente al vecchio stack
- integrazione lidar dentro launch e TF coerenti
- verifica completa dei topic, nomi frame e dipendenze
- consolidamento dei launch file
- validazione end-to-end con hardware reale

### Problema principale attuale

Lo stack ROS2 presente oggi non e' ancora operativo come sistema unico.

Possibili cause da verificare sistematicamente:

- sovrapposizione tra piu' driver ROS2
- mismatch tra protocollo seriale firmware e nodo ROS2 selezionato
- differenze tra messaggi ROS1 legacy e messaggi ROS2 attuali
- topic name non allineati
- frame TF non coerenti
- parametri di encoder, gear ratio e motori non uniformi
- launch parziali o non sincronizzati

## Direzione tecnica consigliata

### Regola principale

Non migrare tutto in parallelo senza una base stabile. Prima si deve consolidare il driver robot ROS2 realmente compatibile col firmware, poi si integrano gli altri nodi.

### Base raccomandata

Usare `5dpo_ratf_driver-main` come base del driver ROS2.

Motivazione:

- e' gia' il componente piu' vicino al comportamento del vecchio stack
- e' coerente col protocollo seriale attuale
- gestisce piu' casi reali di `omnicar_driver`

### Strategia di migrazione

Ordine raccomandato:

1. stabilizzare `sdpo_ratf_driver` e la seriale con ESP32
2. allineare interfacce ROS2 e topic rispetto al legacy
3. portare joystick
4. portare odometria
5. portare localizzazione
6. integrare lidar
7. unificare launch, parametri e TF
8. validare l'intero flusso su robot reale

## Piano di lavoro operativo

### Fase 1: Driver e seriale

Obiettivo:

- ottenere uno scambio affidabile ROS2 <-> ESP32

Verifiche:

- apertura seriale corretta
- ricezione continua dei canali encoder
- pubblicazione corretta di `motors_enc`
- invio corretto di `motors_ref`
- comportamento watchdog
- reset e reconnessione

Deliverable:

- nodo driver ROS2 stabile e unico

### Fase 2: Interfacce ROS2

Obiettivo:

- far corrispondere i messaggi necessari al legacy ROS1

Attivita':

- confrontare `5dpo_ros_interfaces` legacy con `sdpo_drivers_interfaces`
- verificare campi mancanti
- adattare publisher/subscriber e servizi

Deliverable:

- interfacce ROS2 definitive e consistenti

### Fase 3: Joystick

Sorgente legacy:

- `omnicar_ros_ws/src/5dpo_driver_omnijoy`

Obiettivo:

- portare in ROS2 il teleop joystick e riallineare i topic in uscita verso il driver

Deliverable:

- nodo joystick ROS2 funzionante

### Fase 4: Odometria

Sorgente legacy:

- `omnicar_ros_ws/src/5dpo_ros_odom`

Obiettivo:

- calcolare odometria da encoder in ROS2
- pubblicare `odom` e TF coerenti

Deliverable:

- nodo odometria ROS2 con config e launch

### Fase 5: Localizzazione

Sorgente legacy:

- `omnicar_ros_ws/src/5dpo_ratf_ros_localization`
- `omnicar_ros_ws/src/robot_pose_ekf`

Obiettivo:

- ripristinare la pipeline di localizzazione in ROS2

Nota:

- qui conviene valutare se portare direttamente il codice legacy oppure sostituirlo con componenti ROS2 standard dove sensato

Deliverable:

- nodo o pipeline di localizzazione ROS2 integrata

### Fase 6: Lidar

Sorgente legacy:

- `omnicar_ros_ws/src/ldlidar_stl_ros`

Obiettivo:

- far funzionare il lidar in ROS2 con topic, parametri e launch coerenti

Deliverable:

- nodo lidar ROS2 integrato con il resto dello stack

## Decisioni architetturali da mantenere

- una sola implementazione driver robot deve rimanere attiva nel risultato finale
- il protocollo seriale va trattato come contratto di sistema
- i topic ROS2 devono essere stabilizzati prima di costruire sopra joystick/odom/localizzazione
- ogni package migrato deve essere verificato singolarmente e poi integrato nel launch complessivo

## Cose da evitare

- mantenere due driver robot concorrenti
- cambiare insieme firmware, protocollo seriale e nodi ROS2 senza isolare i problemi
- migrare i package legacy senza confrontare topic, parametri e frame originali
- introdurre nuovi messaggi ROS2 senza verificare il loro impatto su tutto lo stack

## Prossimo passo consigliato

Il prossimo passo operativo dovrebbe essere:

- confrontare file per file `5dpo_ratf_ros_driver` e `5dpo_ratf_driver-main`
- identificare le differenze funzionali mancanti nel driver ROS2
- rimuovere o accantonare il percorso `omnicar_driver` se confermato non compatibile
- portare poi in ROS2, nell'ordine, joystick -> odometria -> localizzazione -> lidar

## Nota per chi lavora su questo repository

Quando si apportano modifiche:

- privilegiare `omnicar_ws/` come destinazione finale
- usare `omnicar_ros_ws/` come riferimento funzionale legacy
- considerare `Omnicar/` come sorgente di verita' per il protocollo seriale e per il controllo basso livello
- verificare sempre la compatibilita' end-to-end con hardware reale quando il cambiamento coinvolge driver, seriale o controllori
