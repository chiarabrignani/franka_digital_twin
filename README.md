# Franka Digital Twin

## Descrizione

Progetto di tesi magistrale dedicato alla progettazione e allo sviluppo di un Digital Twin bidirezionale del braccio robotico Franka Emika FR3, simulato in ambiente ROS 2 Jazzy e Gazebo.

Il sistema ha l'obiettivo di mantenere una rappresentazione digitale dello stato articolare del robot e di consentire l'invio di comandi di posizione dal Digital Twin al modello simulato.

L'entanglement tra Digital Twin e robot simulato sarà valutato mediante la metrica ODTE, seguendo la definizione e il metodo di calcolo riportati nella letteratura scientifica di riferimento.

**Nota:** il progetto viene sviluppato e valutato in ambiente simulato. Non è attualmente prevista una validazione sperimentale su un robot fisico reale.

## Obiettivi

- Realizzare una rappresentazione digitale dello stato del robot.
- Ricevere e monitorare le informazioni articolari del FR3.
- Consentire l'invio di comandi di posizione dal Digital Twin al robot simulato.
- Monitorare la regolarità della ricezione degli stati articolari.
- Valutare l'entanglement mediante la metrica ODTE.
- Eseguire test per verificare il corretto funzionamento del sistema.
- Analizzare il comportamento del sistema durante il movimento simultaneo di più giunti.

## Tecnologie utilizzate

| Tecnologia | Utilizzo |
|---|---|
| Ubuntu 24.04 | Sistema operativo |
| ROS 2 Jazzy | Middleware robotico |
| Gazebo | Ambiente di simulazione |
| Python | Implementazione dei nodi ROS 2 |
| Git | Versionamento del codice |
| GitHub | Hosting del repository e documentazione |

## Architettura del sistema

Il progetto è organizzato in componenti ROS 2 distinti, ciascuno responsabile di una specifica funzionalità del Digital Twin.

### 1. State Manager

Lo `State Manager` riceve gli stati articolari del robot attraverso il topic `/joint_states`.

Il componente:

- Identifica i sette giunti del braccio FR3.
- Legge la posizione e la velocità articolare, quando disponibile.
- Aggiorna la rappresentazione digitale dello stato del robot.
- Conserva il timestamp del messaggio sorgente.
- Registra il tempo di ricezione del messaggio.
- Stampa periodicamente lo stato digitale.

### 2. Command Manager

Il `Command Manager` gestisce l'invio dei comandi di posizione dal Digital Twin al robot simulato.

Il componente:

- Riceve i comandi dal topic `/digital_twin/command_positions`.
- Verifica che il comando contenga esattamente sette valori.
- Rifiuta i comandi che contengono valori non finiti.
- Inoltra i comandi validi al controller di posizione del FR3.
- Registra nei log le posizioni obiettivo dei sette giunti.

Il controller utilizzato è `fr3_position_controller`, configurato nell'ambiente ROS 2 e Gazebo.

### 3. Synchronization Monitor

Il `Synchronization Monitor` controlla la regolarità della ricezione degli stati articolari del robot simulato attraverso il topic `/joint_states`.

Il componente:

- Verifica la presenza dei sette giunti richiesti nei messaggi ricevuti.
- Controlla che gli aggiornamenti dello stato articolare siano recenti.
- Segnala il passaggio dallo stato sincronizzato a quello non sincronizzato.
- Rileva il ripristino della ricezione regolare dei dati.
- Registra nei log le transizioni tra i due stati, evitando di ripetere continuamente lo stesso avviso mentre una condizione persiste.

La soglia di obsolescenza dei dati è attualmente impostata a 2 secondi.

Il monitor costituisce un primo meccanismo di controllo della sincronizzazione. Non rappresenta ancora un'implementazione completa della metrica ODTE.

### 4. ODTE Monitor

Il componente dedicato alla metrica ODTE sarà sviluppato nelle successive fasi del progetto.

La sua implementazione seguirà la definizione scientifica di riferimento e utilizzerà le informazioni necessarie per valutare quantitativamente l'entanglement tra il Digital Twin e il robot simulato.

## Comunicazione ROS 2

I principali topic utilizzati dal sistema sono i seguenti.

| Topic | Tipo di messaggio | Funzione |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Pubblicazione degli stati articolari del robot |
| `/digital_twin/command_positions` | `std_msgs/msg/Float64MultiArray` | Ricezione dei comandi di posizione del Digital Twin |
| `/fr3_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Invio dei comandi al controller di posizione |

### Percorso di acquisizione dello stato

```text
FR3 simulato in Gazebo
          |
          v
    /joint_states
          |
          v
      State Manager
          |
          v
Aggiornamento dello stato digitale
          |
          v
 Synchronization Monitor
```

### Percorso di invio dei comandi

```text
Comando del Digital Twin
          |
          v
/digital_twin/command_positions
          |
          v
     Command Manager
          |
          v
/fr3_position_controller/commands
          |
          v
 Controller di posizione
          |
          v
 FR3 simulato in Gazebo
```

## Requisiti

Per eseguire il progetto sono necessari:

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Gazebo configurato per la simulazione del Franka FR3.
- Workspace del robot correttamente compilato.
- Controller `fr3_position_controller` disponibile e attivo per l'invio dei comandi di posizione.
- Package ROS 2 `franka_dt` compilato.

## Installazione e compilazione

Il repository del Digital Twin è organizzato nel package ROS 2 Python `franka_dt`.

Per compilare il package, aprire un terminale ed eseguire:

```bash
cd ~/franka_digital_twin

source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

colcon build --packages-select franka_dt --symlink-install
```

Al termine della compilazione, inizializzare l'ambiente del progetto:

```bash
source ~/franka_digital_twin/install/setup.bash
```

Per verificare gli eseguibili disponibili:

```bash
ros2 pkg executables franka_dt
```

Gli eseguibili del progetto sono:

```text
franka_dt command_manager
franka_dt state_manager
franka_dt synchronization_monitor
franka_dt tracking_test
```

## Avvio del sistema

### 1. Avvio della simulazione

Avviare la simulazione del Franka FR3 in Gazebo utilizzando il launch file disponibile nell'ambiente `franka_ros2_ws`.

Il controller `fr3_position_controller` deve essere attivo prima di inviare comandi di posizione.

### 2. Avvio dello State Manager

Aprire un terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt state_manager
```

Il nodo rimane in ascolto sul topic `/joint_states` e stampa periodicamente lo stato digitale dei sette giunti.

### 3. Avvio del Command Manager

Aprire un secondo terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt command_manager
```

Il nodo rimane in ascolto sul topic `/digital_twin/command_positions` e inoltra al controller i comandi validi ricevuti.

**Attenzione:** prima di inviare comandi, verificare che Gazebo sia in esecuzione e che il controller di posizione sia attivo. I valori inviati devono essere compatibili con i limiti articolari del robot e con la configurazione della simulazione.

### 4. Avvio del Synchronization Monitor

Aprire un terzo terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt synchronization_monitor
```

Il nodo rimane in ascolto sul topic `/joint_states` e controlla la regolarità della ricezione degli stati articolari.

Se non vengono ricevuti messaggi validi oppure se l'ultimo aggiornamento supera la soglia di 2 secondi, il monitor segnala una condizione di mancata sincronizzazione.

Quando la ricezione regolare riprende, il nodo registra il ripristino dello stato sincronizzato.

## Test effettuati

Sono stati eseguiti test per verificare i principali percorsi di comunicazione e il comportamento dei componenti implementati.

### 1. Acquisizione dello stato articolare

È stata verificata la ricezione dei messaggi pubblicati su `/joint_states` e l'aggiornamento della rappresentazione digitale dei sette giunti del braccio.

### 2. Invio dei comandi di posizione

È stata verificata la ricezione di un comando da parte del `Command Manager` e il successivo inoltro al controller di posizione.

Durante il test, la posizione obiettivo del settimo giunto è stata modificata da circa `0.912627344 rad` a `0.922627344 rad`.

Il valore finale rilevato tramite `/joint_states` è risultato corrispondente alla posizione obiettivo richiesta.

### 3. Verifica dello stato digitale

È stato verificato che lo `State Manager` ricevesse il nuovo stato articolare e mostrasse il valore aggiornato del settimo giunto, pari a circa `0.9226 rad`.

### 4. Verifica del Synchronization Monitor

È stato verificato il comportamento del `Synchronization Monitor` durante il normale funzionamento e in seguito all'interruzione della pubblicazione degli stati articolari.

Il test è stato eseguito disattivando temporaneamente il controller `joint_state_broadcaster` e riattivandolo successivamente.

Durante il test è stato verificato che:

- Il monitor segnalasse lo stato sincronizzato in condizioni normali.
- L'interruzione degli aggiornamenti determinasse una segnalazione di mancata sincronizzazione.
- L'avviso non venisse ripetuto continuamente mentre la condizione di mancata sincronizzazione persisteva.
- La riattivazione del controller determinasse il ripristino dello stato sincronizzato.

Il comportamento osservato è risultato coerente con la logica implementata nel monitor.

### 5. Tracking Test: movimento simultaneo dei giunti

È stato implementato il nodo ROS 2 `tracking_test` per verificare il raggiungimento simultaneo dei target di tre giunti del robot Franka FR3.

Il test acquisisce le posizioni iniziali, calcola i target, invia un comando al Digital Twin e verifica il raggiungimento delle posizioni desiderate.

I giunti coinvolti sono:

| Giunto | Variazione richiesta |
|---|---:|
| `fr3_joint2` | `+0.03 rad` |
| `fr3_joint4` | `+0.03 rad` |
| `fr3_joint7` | `-0.03 rad` |

La verifica utilizza una tolleranza di `0.01 rad`. La condizione di raggiungimento deve rimanere stabile per `0.5 s`, con un timeout di `10 s`.

#### Prerequisiti

Prima di eseguire il test, assicurarsi che:

- ROS 2 Jazzy e i workspace siano stati caricati.
- La simulazione Gazebo del FR3 sia attiva.
- Il controller `fr3_position_controller` sia attivo.
- Il `joint_state_broadcaster` sia attivo.
- Il nodo `command_manager` sia in esecuzione.

Caricare gli ambienti ROS 2 e i workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash
```

#### Esecuzione

Eseguire il test in un terminale separato:

```bash
ros2 run franka_dt tracking_test
```

Il nodo acquisisce lo stato iniziale dei sette giunti, invia il comando e verifica il raggiungimento dei target.

Al termine stampa l'esito del test e salva automaticamente le metriche in un file CSV all'interno della cartella `results/`.

#### Metriche registrate

Per ciascun giunto coinvolto vengono registrate le seguenti informazioni:

| Campo CSV | Descrizione |
|---|---|
| `joint` | Nome del giunto |
| `initial_position_rad` | Posizione iniziale |
| `target_position_rad` | Posizione obiettivo |
| `final_position_rad` | Posizione finale |
| `absolute_error_rad` | Errore assoluto tra target e posizione finale |
| `max_observed_error_rad` | Massimo errore osservato durante il movimento |
| `test_success` | Esito del test |
| `elapsed_time_s` | Tempo impiegato per completare il test |

I file vengono salvati con un nome contenente data e ora, ad esempio:

```text
results/tracking_test_20261021_111744.csv
```

Ogni esecuzione genera un file distinto, così da poter confrontare i risultati ottenuti in test differenti.

#### Risultati preliminari

Il test è stato eseguito due volte. In entrambe le esecuzioni l'esito riportato è stato **SUPERATO**, con un tempo di circa `0.601 s`.

Nella prima esecuzione sono stati registrati i seguenti valori:

| Giunto | Errore finale assoluto (rad) | Errore massimo osservato (rad) |
|---|---:|---:|
| `fr3_joint2` | 0.000000 | 0.020751 |
| `fr3_joint4` | 0.000000 | 0.016329 |
| `fr3_joint7` | 0.000000 | 0.009414 |

Gli errori finali sono riportati con sei cifre decimali. Il valore `0.000000` indica che l'errore è inferiore alla precisione mostrata, non necessariamente che sia matematicamente nullo.

Questi risultati costituiscono una prima verifica del movimento simultaneo dei giunti e del raggiungimento dei target. Sono tuttavia preliminari: saranno necessarie ulteriori esecuzioni per valutare la ripetibilità del test e analizzare il comportamento del sistema in condizioni differenti.

I risultati del Tracking Test non costituiscono una valutazione della metrica ODTE, che resta da implementare.

## Stato del progetto

Il progetto si trova nella fase di sviluppo e integrazione dei componenti fondamentali del Digital Twin bidirezionale.

### Attività completate

- Creazione del repository Git.
- Organizzazione iniziale del package ROS 2 `franka_dt`.
- Configurazione del controller di posizione del FR3.
- Implementazione dello `State Manager`.
- Implementazione del `Command Manager`.
- Implementazione del `Synchronization Monitor`.
- Verifica della ricezione degli stati articolari.
- Verifica dell'invio di comandi di posizione al robot simulato.
- Verifica dell'aggiornamento dello stato digitale.
- Verifica della rilevazione della mancata sincronizzazione.
- Verifica del ripristino della sincronizzazione dopo la ripresa della pubblicazione degli stati articolari.
- Implementazione del `Tracking Test` automatico.
- Verifica iniziale del raggiungimento simultaneo dei target di tre giunti.
- Salvataggio delle metriche del test in formato CSV.

### Attività da sviluppare

- Approfondimento della gestione coerente dei riferimenti temporali.
- Implementazione della metrica ODTE secondo la fonte scientifica di riferimento.
- Esecuzione ripetuta e analisi dei test di inseguimento.
- Raccolta e analisi dei risultati ottenuti in simulazione.
- Valutazione della ripetibilità e dei limiti del sistema.
- Documentazione dei risultati e delle limitazioni del Digital Twin.

## Struttura del repository

La struttura principale del repository è organizzata come segue:

```text
franka_digital_twin/
├── README.md
├── .gitignore
├── results/
│   └── tracking_test_*.csv
└── franka_dt/
    ├── franka_dt/
    │   ├── __init__.py
    │   ├── state_manager.py
    │   ├── command_manager.py
    │   ├── synchronization_monitor.py
    │   └── tracking_test.py
    ├── resource/
    ├── test/
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

La cartella `results/` contiene i file CSV generati dalle esecuzioni del Tracking Test. I file di compilazione e installazione di ROS 2 e colcon non sono parte del codice sorgente versionato.

## Versionamento

Il progetto è versionato tramite Git e ospitato su GitHub.

Le modifiche vengono organizzate in commit distinti per documentare l'evoluzione del codice, dell'architettura e delle funzionalità implementate.

Il repository contiene il codice sorgente e la documentazione del progetto. I risultati dei test possono essere conservati localmente nella cartella `results/` e analizzati separatamente.
