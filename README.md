# Franka Digital Twin

## Descrizione

Progetto di tesi magistrale dedicato alla progettazione e allo sviluppo
di un Digital Twin bidirezionale del braccio robotico Franka Emika FR3,
simulato in ambiente ROS 2 Jazzy e Gazebo.

Il sistema ha l'obiettivo di mantenere una rappresentazione digitale
dello stato articolare del robot e di consentire l'invio di comandi
di posizione dal Digital Twin al modello simulato.

L'entanglement tra Digital Twin e robot simulato sarà valutato mediante
la metrica ODTE, seguendo la definizione e il metodo di calcolo
riportati nella letteratura scientifica di riferimento.

**Nota:** il progetto viene sviluppato e valutato in ambiente simulato.
Non è attualmente prevista una validazione sperimentale su un robot
fisico reale.

## Obiettivi

- Realizzare una rappresentazione digitale dello stato del robot.
- Ricevere e monitorare le informazioni articolari del FR3.
- Consentire l'invio di comandi di posizione dal Digital Twin al robot
  simulato.
- Monitorare la sincronizzazione tra il robot simulato e la sua
  rappresentazione digitale.
- Valutare l'entanglement mediante la metrica ODTE.
- Eseguire test per verificare il corretto funzionamento del sistema.

## Tecnologie utilizzate

- **Sistema operativo:** Ubuntu 24.04
- **Middleware robotico:** ROS 2 Jazzy
- **Ambiente di simulazione:** Gazebo
- **Linguaggio di programmazione:** Python
- **Versionamento:** Git e GitHub

## Architettura del sistema

Il progetto è organizzato in componenti ROS 2 distinti, ciascuno
responsabile di una specifica funzionalità del Digital Twin.

### 1. State Manager

Lo `State Manager` riceve gli stati articolari del robot attraverso
il topic `/joint_states`.

Il componente:
- identifica i sette giunti del braccio FR3;
- legge la posizione e la velocità articolare, quando disponibile;
- aggiorna la rappresentazione digitale dello stato del robot;
- conserva il timestamp del messaggio sorgente;
- registra il tempo di ricezione del messaggio;
- stampa periodicamente lo stato digitale.

### 2. Command Manager

Il `Command Manager` gestisce l'invio dei comandi di posizione
dal Digital Twin al robot simulato.

Il componente:
- riceve i comandi dal topic `/digital_twin/command_positions`;
- verifica che il comando contenga esattamente sette valori;
- rifiuta i comandi che contengono valori non finiti;
- inoltra i comandi validi al controller di posizione del FR3;
- registra nei log le posizioni obiettivo dei sette giunti.

Il controller utilizzato è `fr3_position_controller`, configurato
nell'ambiente ROS 2 e Gazebo.

### 3. Componenti pianificati

I seguenti componenti saranno sviluppati nelle successive fasi
del progetto.

**Synchronization Monitor**

Monitorerà la corrispondenza tra lo stato del robot simulato
e quello registrato nella rappresentazione digitale.

**ODTE Monitor**

Implementerà il monitoraggio della metrica ODTE, seguendo la
definizione scientifica di riferimento e utilizzando le informazioni
necessarie per la valutazione dell'entanglement.

## Comunicazione ROS 2

I principali topic utilizzati dal sistema sono i seguenti.

| Topic | Tipo di messaggio | Funzione |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Pubblicazione degli stati articolari del robot |
| `/digital_twin/command_positions` | `std_msgs/msg/Float64MultiArray` | Ricezione dei comandi di posizione del Digital Twin |
| `/fr3_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Invio dei comandi al controller di posizione |

Il percorso di acquisizione dello stato è:

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
```

Il percorso di invio dei comandi è:

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

- Ubuntu 24.04;
- ROS 2 Jazzy;
- Gazebo configurato per la simulazione del Franka FR3;
- workspace del robot correttamente compilato;
- controller `fr3_position_controller` disponibile e attivo;
- package ROS 2 `franka_dt` compilato.

## Installazione e compilazione

Il repository del Digital Twin è organizzato nel package ROS 2
Python `franka_dt`.

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

Gli eseguibili previsti sono:

```text
franka_dt command_manager
franka_dt state_manager
```

## Avvio del sistema

### 1. Avvio della simulazione

Avviare la simulazione del Franka FR3 in Gazebo utilizzando
il launch file disponibile nell'ambiente `franka_ros2_ws`.

Il controller `fr3_position_controller` deve essere attivo
prima di inviare comandi di posizione.

### 2. Avvio dello State Manager

Aprire un terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt state_manager
```

Il nodo rimane in ascolto sul topic `/joint_states` e stampa
periodicamente lo stato digitale dei sette giunti.

### 3. Avvio del Command Manager

Aprire un secondo terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt command_manager
```

Il nodo rimane in ascolto sul topic `/digital_twin/command_positions`
e inoltra al controller i comandi validi ricevuti.

**Attenzione:** prima di inviare comandi, verificare che Gazebo
sia in esecuzione e che il controller di posizione sia attivo.
I valori inviati devono essere compatibili con i limiti articolari
del robot e con la configurazione della simulazione.

## Test effettuati

Sono stati eseguiti test iniziali per verificare i principali
percorsi di comunicazione.

### Acquisizione dello stato articolare

È stata verificata la ricezione dei messaggi pubblicati su
`/joint_states` e l'aggiornamento della rappresentazione digitale
dei sette giunti del braccio.

### Invio dei comandi di posizione

È stata verificata la ricezione di un comando da parte del
`Command Manager` e il successivo inoltro al controller di posizione.

Durante il test, la posizione obiettivo del settimo giunto è stata
modificata da circa `0.912627344 rad` a `0.922627344 rad`.

Il valore finale rilevato tramite `/joint_states` è risultato
corrispondente alla posizione obiettivo richiesta.

### Verifica dello stato digitale

È stato verificato che lo `State Manager` ricevesse il nuovo stato
articolare e mostrasse il valore aggiornato del settimo giunto,
pari a circa `0.9226 rad`.

I test confermano il funzionamento iniziale dei componenti implementati
e dei relativi percorsi di comunicazione in ambiente simulato.

La valutazione quantitativa della sincronizzazione e l'implementazione
della metrica ODTE sono ancora da completare.

## Stato del progetto

Il progetto si trova nella fase di sviluppo e integrazione
dei componenti fondamentali del Digital Twin bidirezionale.

### Attività completate

- Creazione del repository Git.
- Organizzazione iniziale del package ROS 2 `franka_dt`.
- Configurazione del controller di posizione del FR3.
- Implementazione dello `State Manager`.
- Implementazione del `Command Manager`.
- Verifica della ricezione degli stati articolari.
- Verifica dell'invio di comandi di posizione al robot simulato.
- Verifica dell'aggiornamento dello stato digitale.
- Esecuzione dei primi test in Gazebo.

### Attività da sviluppare

- Progettazione e implementazione del `Synchronization Monitor`.
- Definizione di una gestione coerente dei riferimenti temporali.
- Implementazione della metrica ODTE secondo la fonte scientifica
  di riferimento.
- Definizione di test sistematici per la valutazione del sistema.
- Raccolta e analisi dei risultati sperimentali in simulazione.
- Documentazione dei risultati e dei limiti del sistema.

## Struttura del repository

```text
franka_digital_twin/
├── README.md
└── franka_dt/
    ├── franka_dt/
    │   ├── __init__.py
    │   ├── state_manager.py
    │   └── command_manager.py
    ├── resource/
    ├── test/
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

## Versionamento

Il progetto è versionato tramite Git e ospitato su GitHub.

Le modifiche vengono organizzate in commit distinti per documentare
l'evoluzione del codice, dell'architettura e delle funzionalità
implementate.
