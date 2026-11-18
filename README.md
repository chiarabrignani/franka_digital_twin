# Franka Digital Twin

## Descrizione del progetto

Il progetto riguarda lo sviluppo di un Digital Twin del braccio robotico Franka FR3, realizzato con ROS 2 Jazzy e Gazebo.

Il sistema implementa un'architettura bidirezionale che consente sia di acquisire e monitorare lo stato del robot fisico nella sua rappresentazione digitale, sia di inviare comandi dal Digital Twin al robot simulato.

La sincronizzazione tra le due componenti viene analizzata attraverso metriche ODTE (Overall Digital Twin Entanglement), utilizzate per valutare la qualità dello scambio di informazioni.

---

## Obiettivi

Gli obiettivi principali del progetto sono:

- Configurare un ambiente di simulazione del robot Franka FR3 con ROS 2 Jazzy e Gazebo.
- Acquisire, elaborare e monitorare lo stato articolare del robot.
- Implementare una comunicazione bidirezionale tra il robot simulato e il Digital Twin, consentendo lo scambio di stati e comandi.
- Verificare la sincronizzazione e la coerenza delle informazioni scambiate.
- Sviluppare un prototipo per il calcolo delle metriche ODTE nelle direzioni Physical → Digital e Digital → Physical.
- Implementare test automatici e sperimentali per valutare il comportamento e l'affidabilità del sistema.

---

## Tecnologie utilizzate

| Tecnologia | Utilizzo |
|---|---|
| Ubuntu 24.04 | Sistema operativo |
| ROS 2 Jazzy | Middleware per la comunicazione tra i nodi |
| Gazebo | Simulazione del robot Franka FR3 |
| Python | Sviluppo dei nodi ROS 2 e dei test |
| pytest | Esecuzione dei test automatici |
| Git | Controllo di versione |
| GitHub | Hosting del repository |

---

## Architettura del sistema

Il sistema è basato su un'architettura modulare composta da nodi ROS 2, che collaborano per gestire lo scambio di informazioni tra il robot simulato e il Digital Twin.

La comunicazione avviene attraverso topic e Action ROS 2, secondo due direzioni: Physical → Digital e Digital → Physical.

### 1. Digital Command Generator

Il nodo `digital_command_generator.py` genera i comandi di posizione da inviare al robot simulato e li pubblica sul topic `/digital_twin/command_positions`.

Supporta due modalità operative:

- **Tracking:** acquisisce lo stato articolare corrente del robot e genera un nuovo comando applicando variazioni alle posizioni di alcune articolazioni.
- **Full Sequence:** genera una sequenza di comandi di posizione, consentendo di eseguire più movimenti consecutivi.

I comandi generati vengono ricevuti dal `Command Manager`, che ne gestisce l'invio al controller del robot.

### 2. Command Manager

Il nodo `command_manager.py` gestisce l'invio dei comandi di movimento dal Digital Twin al robot simulato. Riceve i comandi generati dal `Digital Command Generator`, ne verifica la validità e li inoltra al controller del robot attraverso l'Action ROS 2 `FollowJointTrajectory`, disponibile su `/fr3_arm_controller/follow_joint_trajectory`.

Il nodo gestisce inoltre l'accettazione e l'esecuzione dei comandi, monitorandone l'esito e gli eventuali timeout. Pubblica gli aggiornamenti sullo stato dei comandi attraverso `/digital_twin/command_status`.

Infine, genera un heartbeat periodico sul topic `/digital_twin/physical_heartbeat`, utilizzato per monitorare la disponibilità della comunicazione con il robot.

### 3. State Manager

Il nodo `state_manager.py` acquisisce lo stato articolare del robot dal topic `/joint_states` e ne verifica la validità, controllando la presenza delle sette articolazioni e la coerenza dei valori ricevuti.

Una volta elaborati i dati, pubblica lo stato articolare aggiornato sul topic `/digital_twin/joint_states`, mantenendo un ordine deterministico dei giunti.

Gestisce inoltre le informazioni temporali associate agli aggiornamenti, utilizzate per monitorare la comunicazione nella direzione Physical → Digital.

### 4. Diagnostica della sincronizzazione PT-DT

Il nodo `odte_monitor.py` si occupa di monitorare la sincronizzazione tra Physical Twin (PT) e Digital Twin (DT), confrontando gli stati articolari ricevuti attraverso `/joint_states` e `/digital_twin/predicted_joint_states`.

Gli aggiornamenti vengono associati in base ai rispettivi timestamp, così da confrontare stati temporalmente compatibili. Per ogni confronto vengono calcolati gli errori articolari e le relative statistiche, tra cui l'errore medio e quello massimo.

Il nodo segnala inoltre eventuali problemi di sincronizzazione o indisponibilità dei dati. Queste informazioni hanno finalità diagnostiche e sono mantenute distinte dal calcolo delle metriche ODTE.

### 5. Digital Model

Il nodo `digital_model.py` elabora gli aggiornamenti dello stato articolare ricevuti dal Digital Twin e produce una previsione dello stato articolare.

Pubblica i risultati attraverso `/digital_twin/predicted_joint_states` e le informazioni relative all'elaborazione attraverso `/digital_twin/state_processed`.

### 6. ODTE Metrics e ODTE Monitor

Il modulo `odte_metrics.py` contiene le funzioni matematiche di supporto al calcolo delle componenti della metrica ODTE:

- **Timeliness (T):** tempestività delle informazioni.
- **Reliability (R):** affidabilità dello scambio di informazioni.
- **Availability (A):** disponibilità della comunicazione.

Il nodo `odte_monitor.py` raccoglie le informazioni provenienti dai diversi componenti del sistema e calcola le metriche per entrambe le direzioni di comunicazione.

La metrica complessiva viene calcolata attraverso:

```text
ODTE = T × R × A
```

L'implementazione costituisce un prototipo sperimentale, ancora soggetto a verifiche e validazione.

---

## Architettura della comunicazione

Il sistema prevede due flussi principali.

### Physical → Digital (P→D)

Il robot simulato pubblica il proprio stato articolare attraverso `/joint_states`. Lo State Manager acquisisce e valida i dati, pubblicandoli su `/digital_twin/joint_states`.

Il Digital Model elabora successivamente le informazioni ricevute e pubblica lo stato predetto e i relativi dati temporali, utilizzati dal monitor ODTE.

### Digital → Physical (D→P)

Il Digital Command Generator genera i comandi di posizione e li pubblica su `/digital_twin/command_positions`.

Il Command Manager riceve i comandi e li inoltra al controller del robot attraverso l'Action `FollowJointTrajectory`.

L'esito dell'esecuzione viene comunicato attraverso `/digital_twin/command_status`, mentre l'heartbeat contribuisce al monitoraggio della disponibilità della comunicazione.

## Visualizzazione del Digital Twin in RViz

Il sistema prevede una visualizzazione in RViz delle due rappresentazioni del robot Franka FR3: il Physical Twin e il Digital Twin.

I due bracci vengono visualizzati contemporaneamente, consentendo di osservare il loro allineamento e confrontarne gli stati articolari.

Durante l'esecuzione di un movimento, entrambe le rappresentazioni devono aggiornarsi e seguire il movimento, mantenendo la corrispondenza tra le rispettive posizioni articolari.

La visualizzazione comprende inoltre una tabella dedicata alle metriche ODTE, che permette di monitorare le principali componenti della comunicazione:

- **Timeliness (T):** tempestività dello scambio di informazioni.
- **Reliability (R):** affidabilità della comunicazione.
- **Availability (A):** disponibilità della comunicazione.
- **ODTE:** valore complessivo ottenuto dalla combinazione delle tre componenti.

Le metriche vengono monitorate separatamente per le direzioni Physical → Digital e Digital → Physical, consentendo di osservare il comportamento del sistema durante il funzionamento.

---

## Topic e Action ROS 2

I principali canali di comunicazione utilizzati dal sistema sono:

| Topic / Action | Tipo | Funzione |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Stato articolare del robot |
| `/digital_twin/joint_states` | `sensor_msgs/msg/JointState` | Stato articolare acquisito dal Digital Twin |
| `/digital_twin/command_positions` | `std_msgs/msg/Float64MultiArray` | Comandi di posizione generati dal Digital Twin |
| `/digital_twin/command_status` | `std_msgs/msg/String` | Stato dei comandi |
| `/digital_twin/physical_heartbeat` | `std_msgs/msg/Bool` | Heartbeat per il monitoraggio della disponibilità |
| `/digital_twin/predicted_joint_states` | `sensor_msgs/msg/JointState` | Stato articolare predetto |
| `/digital_twin/state_processed` | `std_msgs/msg/String` | Informazioni sull'elaborazione dello stato |
| `/digital_twin/odte` | `std_msgs/msg/String` | Risultati delle metriche ODTE |
| `/fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Invio dei comandi al controller |

---

## Requisiti

Per eseguire il progetto sono necessari:

- Ubuntu 24.04.
- ROS 2 Jazzy.
- Gazebo e i pacchetti necessari alla simulazione del Franka FR3.
- Python e gli strumenti di compilazione ROS 2.
- Il workspace `franka_ros2_ws`, contenente i componenti per la simulazione.
- Il repository `franka_digital_twin`.
- `pytest` per l'esecuzione dei test automatici.

---

## Compilazione del progetto

Caricare gli ambienti ROS 2 necessari:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

Spostarsi nella cartella del repository:

```bash
cd ~/franka_digital_twin
```

Compilare il package:

```bash
colcon build --packages-select franka_dt --symlink-install
```

Caricare l'ambiente del progetto:

```bash
source ~/franka_digital_twin/install/setup.bash
```

Nei nuovi terminali sarà necessario caricare gli ambienti ROS 2 e i workspace utilizzati.

---

## Avvio del sistema

Il sistema dispone di un launch file unificato:

```bash
ros2 launch franka_dt digital_twin_system.launch.py
```

Il launch file coordina l'avvio dei componenti previsti dall'architettura del Digital Twin.

---

## Generazione dei comandi

Il Digital Command Generator permette di generare comandi attraverso il servizio:

```text
/digital_twin/generate_command
```

Per attivare la modalità Tracking:

```bash
ros2 service call /digital_twin/generate_command franka_dt_interfaces/srv/GenerateCommand "{mode: 'tracking', step_index: 0}"
```

La modalità Full Sequence consente invece di avviare una sequenza di movimenti predefiniti.

---

## Test e validazione

Il progetto comprende verifiche funzionali e sperimentali relative ai principali componenti.

### Verifica dello stato articolare

Sono state verificate l'acquisizione dello stato del robot attraverso `/joint_states` e la pubblicazione dei dati elaborati dal Digital Twin attraverso `/digital_twin/joint_states`.

### Verifica dei comandi

Sono state effettuate prove di generazione e invio dei comandi al robot simulato, verificando l'accettazione e l'esito delle richieste.

### Verifica della sincronizzazione PT-DT

È stata implementata una diagnostica basata sul confronto temporale e articolare tra gli stati del Physical Twin e del Digital Twin.

### Verifica delle modalità operative

Sono previste due modalità di generazione dei comandi:

- **Tracking:** applicazione di variazioni alle posizioni articolari selezionate.
- **Full Sequence:** esecuzione di una sequenza di movimenti sulle articolazioni del robot.

### Verifica dell'heartbeat

È stato verificato il comportamento dell'heartbeat in presenza di interruzioni e successivo ripristino degli aggiornamenti dello stato articolare.

### Verifica delle metriche ODTE

Il monitor ODTE è stato avviato per entrambe le direzioni di comunicazione. Sono state verificate la ricezione degli eventi, l'elaborazione delle informazioni e la pubblicazione dei risultati.

L'implementazione rimane sperimentale: i valori ottenuti non costituiscono ancora una validazione scientifica della metrica originale.

### Verifica della visualizzazione in RViz

È stata predisposta una visualizzazione simultanea del Physical Twin e del Digital Twin in RViz.

La verifica consiste nel controllare che:

- Entrambe le rappresentazioni del robot siano visualizzate correttamente.
- I due bracci risultino inizialmente allineati.
- Durante l'esecuzione dei comandi, entrambi seguano il movimento mantenendo la corrispondenza tra le posizioni articolari.
- La tabella ODTE mostri le metriche relative a entrambe le direzioni di comunicazione.

Questa visualizzazione permette di affiancare al monitoraggio numerico delle metriche una verifica visiva del comportamento del Digital Twin.

---

## Test automatici

Il package include test automatici realizzati con `pytest`.

Per eseguirli:

```bash
cd ~/franka_digital_twin/franka_dt
python3 -m pytest test -v
```

I test verificano il comportamento dei principali componenti e delle funzioni di supporto del sistema.

---

## Struttura del repository

La struttura principale del progetto è organizzata come segue:

```text
franka_digital_twin/
├── README.md
├── docs/
│   └── architecture.md
├── franka_dt/
│   ├── franka_dt/
│   │   ├── command_manager.py
│   │   ├── digital_command_generator.py
│   │   ├── digital_model.py
│   │   ├── state_manager.py
│   │   ├── odte_monitor.py
│   │   └── odte_metrics.py
│   ├── test/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── .gitignore
```

La cartella `docs/` contiene la documentazione tecnica, mentre la cartella `test/` raccoglie i test automatici.

---

## Stato del progetto

### Funzionalità implementate

- Acquisizione e pubblicazione dello stato articolare.
- Generazione dei comandi nelle modalità Tracking e Full Sequence.
- Gestione dei comandi attraverso Action ROS 2.
- Monitoraggio dell'esecuzione dei comandi.
- Pubblicazione dell'heartbeat.
- Elaborazione e previsione dello stato articolare tramite Digital Model.
- Diagnostica della sincronizzazione PT-DT.
- Calcolo sperimentale delle metriche ODTE nelle due direzioni.
- Test automatici e verifiche sperimentali.

### Sviluppi futuri

- Consolidare la validazione della metrica ODTE rispetto alla formulazione scientifica di riferimento.
- Approfondire il calcolo di Timeliness, Reliability e Availability.
- Ampliare le campagne sperimentali sulle tolleranze e sui parametri temporali.
- Verificare la robustezza della comunicazione bidirezionale.
- Estendere i test automatici e funzionali.
- Completare la documentazione tecnica del sistema.

---

## Documentazione

La documentazione tecnica dell'architettura è disponibile in:

```text
docs/architecture.md
```

---

## Controllo di versione

Il progetto utilizza Git per il controllo di versione e GitHub per ospitare e condividere il codice sorgente.

---

## Riferimento bibliografico

La metrica ODTE prende come riferimento il seguente lavoro scientifico:

Paolo Bellavista et al., ODTE: A Metric for Digital Twin Entanglement, IEEE Open Journal of the Communications Society, 2024.
