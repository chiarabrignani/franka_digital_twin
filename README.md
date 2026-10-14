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
- Monitorare la regolarità della ricezione degli stati articolari.
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

### 3. Synchronization Monitor

Il `Synchronization Monitor` controlla la regolarità della ricezione
degli stati articolari del robot simulato attraverso il topic
`/joint_states`.

Il componente:
- verifica la presenza dei sette giunti richiesti nei messaggi ricevuti;
- controlla che gli aggiornamenti dello stato articolare siano recenti;
- segnala il passaggio dallo stato sincronizzato a quello non sincronizzato;
- rileva il ripristino della ricezione regolare dei dati;
- registra nei log le transizioni tra i due stati, evitando di ripetere
  continuamente lo stesso avviso mentre una condizione persiste.

La soglia di obsolescenza dei dati è attualmente impostata a 2 secondi.

Il monitor costituisce un primo meccanismo di controllo della
sincronizzazione. Non rappresenta ancora un'implementazione completa
della metrica ODTE.

### 4. ODTE Monitor

Il componente dedicato alla metrica ODTE sarà sviluppato nelle
successive fasi del progetto.

La sua implementazione seguirà la definizione scientifica di
riferimento e utilizzerà le informazioni necessarie per valutare
quantitativamente l'entanglement tra il Digital Twin e il robot
simulato.

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
          |
          v
 Synchronization Monitor
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
- controller `fr3_position_controller` disponibile e attivo per l'invio
  dei comandi di posizione;
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

Gli eseguibili disponibili sono:

```text
franka_dt command_manager
franka_dt state_manager
franka_dt synchronization_monitor
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

### 4. Avvio del Synchronization Monitor

Aprire un terzo terminale ed eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash

ros2 run franka_dt synchronization_monitor
```

Il nodo rimane in ascolto sul topic `/joint_states` e controlla
periodicamente la regolarità della ricezione degli stati articolari.

Se non vengono ricevuti messaggi validi oppure se l'ultimo
aggiornamento supera la soglia di 2 secondi, il monitor segnala
una condizione di mancata sincronizzazione.

Quando la ricezione regolare riprende, il nodo registra il
ripristino dello stato sincronizzato.

## Test effettuati

Sono stati eseguiti test per verificare i principali percorsi
di comunicazione e il comportamento dei componenti implementati.

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

### Verifica del Synchronization Monitor

È stato verificato il comportamento del `Synchronization Monitor`
durante il normale funzionamento e in seguito all'interruzione
della pubblicazione degli stati articolari.

Il test è stato eseguito disattivando temporaneamente il controller
`joint_state_broadcaster` e riattivandolo successivamente.

Durante il test è stato verificato che:
- il monitor segnalasse lo stato sincronizzato in condizioni normali;
- l'interruzione degli aggiornamenti determinasse una segnalazione
  di mancata sincronizzazione;
- l'avviso non venisse ripetuto continuamente mentre la condizione
  di mancata sincronizzazione persisteva;
- la riattivazione del controller determinasse il ripristino
  dello stato sincronizzato.

Il comportamento osservato è risultato coerente con la logica
implementata nel monitor.

Il test è stato eseguito nell'ambiente simulato ROS 2 e Gazebo.

I test confermano il funzionamento dei componenti implementati
e dei relativi percorsi di comunicazione in ambiente simulato.

Il `Synchronization Monitor` consente di rilevare le interruzioni
nella ricezione degli stati articolari e il successivo ripristino
degli aggiornamenti.

La valutazione quantitativa dell'entanglement mediante la metrica
ODTE è ancora da implementare.

## Stato del progetto

Il progetto si trova nella fase di sviluppo e integrazione
dei componenti fondamentali del Digital Twin bidirezionale.

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
- Verifica del ripristino della sincronizzazione dopo la ripresa
  della pubblicazione degli stati articolari.
- Esecuzione dei test iniziali in Gazebo.

### Attività da sviluppare

- Approfondimento della gestione coerente dei riferimenti temporali.
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
    │   ├── command_manager.py
    │   └── synchronization_monitor.py
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
