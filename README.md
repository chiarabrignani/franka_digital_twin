# Franka Digital Twin

## Descrizione del progetto

Il progetto mira alla realizzazione di un **Digital Twin del robot Franka FR3**, utilizzando ROS 2 Jazzy e Gazebo per la simulazione e lo sviluppo di un sistema in grado di acquisire lo stato del robot, inviare comandi di movimento e monitorare la sincronizzazione tra il sistema fisico e la sua rappresentazione digitale.

L'attività si concentra sulla progettazione e sull'implementazione di un'architettura modulare basata su nodi ROS 2, con particolare attenzione alla comunicazione, al controllo delle posizioni articolari e al monitoraggio della sincronizzazione.

Il progetto comprende inoltre un primo prototipo sperimentale per il calcolo della metrica **ODTE (Overall Digital Twin Entanglement)**, utilizzata per esplorare la qualità dell'interazione e dell'aggiornamento tra il sistema fisico e il Digital Twin.

---

## Obiettivi

Gli obiettivi principali del progetto sono:

- Configurare l'ambiente ROS 2 Jazzy e Gazebo per simulare il robot Franka FR3.
- Acquisire e monitorare lo stato delle articolazioni del robot.
- Implementare un nodo per la gestione e l'inoltro dei comandi di posizione.
- Monitorare la disponibilità degli aggiornamenti dello stato del robot.
- Verificare la capacità del sistema di raggiungere simultaneamente determinati valori di posizione articolare.
- Sviluppare un prototipo sperimentale per il calcolo della metrica ODTE.
- Organizzare il codice in moduli distinti e mantenere il progetto sotto controllo di versione tramite Git.

---

## Tecnologie utilizzate

| Tecnologia | Utilizzo |
|---|---|
| Ubuntu 24.04 | Sistema operativo |
| ROS 2 Jazzy | Middleware per la comunicazione tra i nodi |
| Gazebo | Simulazione del robot Franka FR3 |
| Python | Implementazione dei nodi ROS 2 e dei test |
| Git | Controllo di versione |
| GitHub | Repository del progetto |

---

## Architettura del sistema

Il sistema è organizzato in diversi nodi ROS 2, ciascuno con una responsabilità specifica.

### 1. State Manager

Il nodo `state_manager.py` acquisisce lo stato del robot attraverso il topic `/joint_states`.

Le principali responsabilità sono:

- Ricevere gli aggiornamenti relativi alle articolazioni del robot.
- Mantenere lo stato corrente delle sette articolazioni del FR3.
- Visualizzare informazioni sullo stato acquisito.
- Fornire una base per il monitoraggio e per le successive elaborazioni.

### 2. Command Manager

Il nodo `command_manager.py` gestisce l'invio dei comandi di posizione al robot.

Riceve i comandi attraverso il topic `/digital_twin/command_positions` e li inoltra al controller attraverso il topic `/fr3_position_controller/commands`.

Le principali responsabilità sono:

- Ricevere un array contenente le posizioni articolari desiderate.
- Verificare che il comando contenga esattamente sette valori.
- Verificare che i valori siano numerici e finiti.
- Inoltrare i comandi validi al controller del robot.

### 3. Synchronization Monitor

Il nodo `synchronization_monitor.py` controlla la freschezza degli aggiornamenti ricevuti attraverso `/joint_states`.

Il monitor utilizza una soglia di **2 secondi** per individuare eventuali interruzioni o ritardi significativi negli aggiornamenti.

Le principali responsabilità sono:

- Monitorare la ricezione degli aggiornamenti.
- Verificare se lo stato del robot è aggiornato.
- Rilevare le transizioni tra uno stato sincronizzato e uno non sincronizzato.
- Segnalare eventuali interruzioni nella ricezione dei dati.

### 4. Tracking Test

Il modulo `tracking_test.py` verifica la capacità del sistema di raggiungere simultaneamente determinate posizioni articolari.

Il test coinvolge le articolazioni:

- `fr3_joint2`
- `fr3_joint4`
- `fr3_joint7`

Le posizioni obiettivo vengono ottenute applicando le seguenti variazioni:

| Articolazione | Variazione richiesta |
|---|---:|
| `fr3_joint2` | +0.03 rad |
| `fr3_joint4` | +0.03 rad |
| `fr3_joint7` | -0.03 rad |

I parametri utilizzati per il test sono:

| Parametro | Valore |
|---|---:|
| Tolleranza sulla posizione | 0.01 rad |
| Durata minima di stabilità | 0.5 s |
| Timeout | 10 s |

Il test verifica che tutte le articolazioni coinvolte raggiungano i rispettivi obiettivi entro la tolleranza prevista e rimangano stabili per il tempo richiesto.

I risultati vengono salvati in un file CSV nella cartella `results/`.

### 5. ODTE Monitor

Il modulo `odte_monitor.py` implementa un primo prototipo sperimentale per il monitoraggio della metrica ODTE.

Il nodo:

- Si sottoscrive al topic `/joint_states`.
- Verifica la presenza delle sette articolazioni richieste.
- Controlla che le posizioni ricevute siano finite.
- Elabora le informazioni temporali e gli aggiornamenti disponibili.
- Calcola i valori delle componenti della metrica.
- Pubblica i risultati in formato JSON attraverso `/digital_twin/odte`.

Il messaggio pubblicato utilizza il tipo ROS 2 `std_msgs/msg/String`.

**Nota importante:** questa implementazione è sperimentale e non costituisce ancora una riproduzione scientificamente validata della metrica ODTE descritta in letteratura. In particolare, alcune componenti sono stimate in modo preliminare e richiedono ulteriori verifiche.

Le limitazioni sono descritte nella sezione dedicata ai test ODTE.

---

## Architettura della comunicazione

### Acquisizione dello stato

Il flusso di acquisizione dello stato del robot è il seguente:

```text
Gazebo / Robot FR3
        |
        | /joint_states
        v
   State Manager
        |
        +----------------------+
        |                      |
        v                      v
Synchronization Monitor   ODTE Monitor
                               |
                               | /digital_twin/odte
                               v
                         Metriche ODTE
```

### Invio dei comandi

Il flusso di invio dei comandi è il seguente:

```text
Comando desiderato
        |
        | /digital_twin/command_positions
        v
   Command Manager
        |
        | /fr3_position_controller/commands
        v
   FR3 Position Controller
        |
        v
      Robot FR3
```

---

## Topic ROS 2

I principali topic utilizzati dal progetto sono riportati nella tabella seguente.

| Topic | Tipo di messaggio | Funzione |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Pubblicazione dello stato delle articolazioni |
| `/digital_twin/command_positions` | `std_msgs/msg/Float64MultiArray` | Ricezione dei comandi di posizione desiderati |
| `/fr3_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Invio dei comandi al controller |
| `/digital_twin/odte` | `std_msgs/msg/String` | Pubblicazione delle metriche ODTE in formato JSON |

---

## Requisiti

Per eseguire il progetto è necessario disporre di:

- Ubuntu 24.04.
- ROS 2 Jazzy installato e configurato.
- Un workspace contenente i componenti ROS 2 necessari per la simulazione del Franka FR3.
- Gazebo e i pacchetti di simulazione del robot.
- Python e gli strumenti di compilazione ROS 2.

Il progetto utilizza il workspace `franka_ros2_ws` per i componenti di simulazione e il workspace `franka_digital_twin` per il package sviluppato.

---

## Compilazione del progetto

Prima di compilare il package, caricare gli ambienti ROS 2 necessari.

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

Spostarsi nella cartella del repository:

```bash
cd ~/franka_digital_twin
```

Compilare il package `franka_dt`:

```bash
colcon build --packages-select franka_dt --symlink-install
```

Dopo la compilazione, caricare l'ambiente del package:

```bash
source ~/franka_digital_twin/install/setup.bash
```

Per utilizzare i nodi del progetto in un nuovo terminale, caricare nuovamente gli ambienti necessari:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash
```

---

## Avvio della simulazione

Per avviare la simulazione del Franka FR3 con il controller di posizione e la mano del robot, eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py load_gripper:=true franka_hand:=franka_hand controller:=fr3_position_controller
```

Una volta avviata la simulazione, verificare che il topic `/joint_states` sia disponibile:

```bash
ros2 topic list
```

Per visualizzare gli aggiornamenti dello stato articolare:

```bash
ros2 topic echo /joint_states
```

---

## Esecuzione dei nodi

Prima di eseguire ciascun nodo in un nuovo terminale, caricare gli ambienti ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_digital_twin/install/setup.bash
```

### State Manager

Avvia il nodo responsabile dell'acquisizione dello stato articolare:

```bash
ros2 run franka_dt state_manager
```

### Command Manager

Avvia il nodo responsabile della gestione dei comandi di posizione:

```bash
ros2 run franka_dt command_manager
```

### Synchronization Monitor

Avvia il monitor della freschezza degli aggiornamenti:

```bash
ros2 run franka_dt synchronization_monitor
```

### ODTE Monitor

Avvia il prototipo sperimentale per il calcolo della metrica ODTE:

```bash
ros2 run franka_dt odte_monitor --ros-args -p use_sim_time:=true -p expected_rate_hz:=100.0 -p desired_timeliness_ms:=50.0 -p window_seconds:=10.0
```

Il nodo utilizza il tempo di simulazione ROS attraverso `use_sim_time:=true`.

I parametri configurabili sono:

| Parametro | Valore utilizzato | Descrizione |
|---|---:|---|
| `use_sim_time` | `true` | Utilizza il tempo di simulazione ROS |
| `expected_rate_hz` | `100.0` | Frequenza di aggiornamento attesa |
| `desired_timeliness_ms` | `50.0` | Soglia temporale desiderata, espressa in millisecondi |
| `window_seconds` | `10.0` | Finestra temporale utilizzata per il monitoraggio |

Per osservare i risultati pubblicati:

```bash
ros2 topic echo /digital_twin/odte
```

I dati vengono pubblicati come messaggi `std_msgs/msg/String` contenenti informazioni in formato JSON.

---

## Test e risultati

### 1. Verifica dell'acquisizione dello stato

L'acquisizione dello stato è stata verificata attraverso il topic `/joint_states`.

Il topic fornisce informazioni sulle articolazioni del robot, utilizzate dai nodi del progetto per monitorare lo stato corrente e verificare il raggiungimento delle posizioni desiderate.

### 2. Verifica del Command Manager

È stata verificata la possibilità di inviare un comando di posizione attraverso `/digital_twin/command_positions` e inoltrarlo al controller.

Durante una delle prove, la posizione di riferimento di `fr3_joint7` è stata modificata da:

```text
0.912627344 rad
```

a:

```text
0.922627344 rad
```

La variazione richiesta è pari a:

```text
+0.01 rad
```

Il test è stato utilizzato per verificare il percorso del comando dal nodo di gestione fino al controller.

### 3. Verifica del Synchronization Monitor

Il monitor è stato verificato simulando un'interruzione degli aggiornamenti dello stato articolare.

La procedura prevede l'arresto e il successivo riavvio del `joint_state_broadcaster`, in modo da osservare il comportamento del monitor quando gli aggiornamenti non arrivano più e quando riprendono.

La soglia utilizzata per identificare uno stato non aggiornato è di 2 secondi.

### 4. Tracking Test

Il test di tracking verifica il raggiungimento simultaneo delle posizioni desiderate per `fr3_joint2`, `fr3_joint4` e `fr3_joint7`.

Le variazioni richieste sono:

| Articolazione | Variazione |
|---|---:|
| `fr3_joint2` | +0.03 rad |
| `fr3_joint4` | +0.03 rad |
| `fr3_joint7` | -0.03 rad |

I parametri di verifica sono:

| Parametro | Valore |
|---|---:|
| Tolleranza | 0.01 rad |
| Stabilità richiesta | 0.5 s |
| Timeout massimo | 10 s |

Sono state completate due esecuzioni del test con esito positivo, con un tempo di completamento di circa 0.601 secondi.

In una delle esecuzioni, gli errori assoluti finali riportati sono stati:

| Articolazione | Errore assoluto finale |
|---|---:|
| `fr3_joint2` | 0.000000 rad |
| `fr3_joint4` | 0.000000 rad |
| `fr3_joint7` | 0.000000 rad |

Gli errori massimi registrati durante il test sono stati:

| Articolazione | Errore massimo |
|---|---:|
| `fr3_joint2` | 0.020751 rad |
| `fr3_joint4` | 0.016329 rad |
| `fr3_joint7` | 0.009414 rad |

Il test ha quindi verificato il raggiungimento degli obiettivi entro la tolleranza prevista e il mantenimento della stabilità per il tempo richiesto.

I risultati vengono salvati in un file CSV nella cartella `results/`.

### 5. ODTE Monitor: risultati preliminari

È stato avviato il nodo sperimentale `odte_monitor.py`, che pubblica i risultati sul topic `/digital_twin/odte`.

Durante l'esecuzione sono stati osservati messaggi di questo tipo:

```text
[INFO] ODTE Monitor avviato: percorso Physical -> Digital.
[WARN] ODTE non ancora calcolabile: servono almeno due aggiornamenti validi.
[INFO] T=1.000 | R=0.006 | A=1.000 | ODTE=0.006
[INFO] T=1.000 | R=1.000 | A=1.000 | ODTE=1.000
```

Il primo avviso indica che, all'avvio, il monitor necessita di almeno due aggiornamenti validi prima di poter effettuare il calcolo.

Successivamente, il nodo ha iniziato a pubblicare messaggi JSON sul topic `/digital_twin/odte`, contenenti le informazioni relative alla direzione del flusso, alla tempestività, all'affidabilità, alla disponibilità e al valore complessivo ODTE.

Un esempio della struttura dei dati pubblicati è:

```json
{
  "direction": "physical_to_digital",
  "timeliness": 1.0,
  "reliability": 1.0,
  "availability": 1.0,
  "odte": 1.0
}
```

Questo JSON illustra la struttura del messaggio osservato e non rappresenta un risultato di validazione scientifica della metrica.

#### Precisazioni sull'implementazione ODTE

Il monitor costituisce un **prototipo sperimentale**. I risultati ottenuti finora dimostrano che il nodo si avvia, riceve gli aggiornamenti e pubblica i valori calcolati, ma non dimostrano ancora che la metrica sia implementata in modo scientificamente equivalente alla formulazione originale.

In particolare:

- **Timeliness (`T`)**: viene stimata attraverso le informazioni temporali disponibili, confrontando il timestamp del messaggio con il tempo ROS utilizzato dal monitor e considerando la soglia temporale configurata. La correttezza della stima dipende dalla coerenza dei timestamp e dalla gestione del tempo di simulazione.

- **Reliability (`R`)**: viene stimata in funzione della frequenza di aggiornamento osservata rispetto alla frequenza attesa, con il risultato limitato a un massimo pari a 1. Il valore può essere inferiore a 1 durante l'avvio o in presenza di aggiornamenti insufficienti.

- **Availability (`A`)**: nella versione attuale è impostata a 1. Non è quindi una misura indipendente della disponibilità del sistema.

- **Execution time**: la misura attuale del tempo di esecuzione non rappresenta ancora una misura attendibile del tempo necessario per elaborare un aggiornamento completo del Digital Twin. La misurazione deve essere rivista per includere l'intervallo effettivo di elaborazione.

- **Valore ODTE**: viene calcolato combinando le componenti implementate. Il risultato deve essere interpretato tenendo conto delle limitazioni sopra descritte.

Di conseguenza, un valore `ODTE = 1.0` non dimostra, da solo, una sincronizzazione perfetta tra il robot e il Digital Twin.

Per rendere il monitor più rigoroso sarà necessario verificare la formulazione matematica originale, definire chiaramente i timestamp di riferimento, misurare il tempo di elaborazione effettivo e validare separatamente ciascuna componente della metrica.

---

## Struttura del repository

La struttura prevista del repository è la seguente:

```text
franka_digital_twin/
├── README.md
├── docs/
│   └── architecture.md
├── .gitignore
└── franka_dt/
    ├── franka_dt/
    │   ├── __init__.py
    │   ├── state_manager.py
    │   ├── command_manager.py
    │   ├── synchronization_monitor.py
    │   ├── tracking_test.py
    │   └── odte_monitor.py
    ├── resource/
    ├── test/
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

La struttura mostra l'organizzazione prevista del repository. Le cartelle e i file effettivamente presenti possono variare in base allo stato corrente del progetto.

---

## Stato attuale del progetto

### Funzionalità implementate

- Configurazione dell'ambiente ROS 2 Jazzy e Gazebo per il robot Franka FR3.
- Acquisizione dello stato articolare tramite `/joint_states`.
- Gestione e inoltro dei comandi di posizione.
- Monitoraggio della freschezza degli aggiornamenti dello stato.
- Test del raggiungimento simultaneo di posizioni articolari.
- Salvataggio dei risultati dei test di tracking in formato CSV.
- Primo prototipo di monitoraggio ODTE con pubblicazione di messaggi JSON.

### Sviluppi futuri

- Consolidare la documentazione dell'architettura.
- Ampliare i test automatici dei nodi ROS 2.
- Migliorare la gestione e la validazione dei comandi.
- Valutare quantitativamente la sincronizzazione tra robot e Digital Twin.
- Verificare la formulazione matematica originale della metrica ODTE.
- Migliorare il calcolo di timeliness, reliability e availability.
- Misurare correttamente il tempo di elaborazione del Digital Twin.
- Validare sperimentalmente i risultati della metrica ODTE in scenari differenti.

---

## Documentazione

La documentazione dell'architettura del sistema è organizzata nel file:

```text
docs/architecture.md
```

Il documento descrive l'organizzazione dei componenti, i flussi di comunicazione e le responsabilità dei nodi principali.

---

## Controllo di versione

Il progetto utilizza **Git** per il controllo di versione e **GitHub** per ospitare il repository e condividere il codice sorgente.

---

## Nota conclusiva

Il progetto costituisce una base per lo sviluppo di un Digital Twin del Franka FR3 in ambiente ROS 2 e Gazebo.

Le funzionalità di acquisizione dello stato, gestione dei comandi e tracking sono state implementate e testate negli scenari descritti. Il monitor ODTE è invece da considerarsi una componente sperimentale, che richiede ulteriori verifiche teoriche e sperimentali prima di poter essere utilizzata come misura validata della qualità della sincronizzazione.
