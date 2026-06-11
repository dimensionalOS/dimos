# Diagramas de arquitectura y secuencia

Ultima modificacion: 2026-06-11 12:04:30 -05 -0500

Los diagramas representan la **arquitectura propuesta**. No describen por si
solos el estado actual implementado en DimOS.

## 1. Arquitectura total

```mermaid
flowchart TB
    subgraph Interfaces
        MIC[Microfono]
        TXT[Texto/API]
        UI[Consola del operador]
    end

    subgraph Cognicion
        ASR[ASR + VAD]
        LLM[Agente]
        ORQ[Orquestador]
        TTS[TTS]
    end

    subgraph Autonomia
        PER[Percepcion]
        SEM[Memoria semantica]
        MAP[Mapas]
        NAV[Navegacion]
    end

    subgraph Seguridad
        SUP[Supervisor]
        ARB[Arbitro]
        WD[Watchdog]
    end

    subgraph Robot
        ADP[Adaptador Unitree]
        G1[G1]
        SENS[RGB-D + Mid-360 + IMU]
    end

    MIC --> ASR --> LLM
    TXT --> LLM
    UI --> ORQ
    LLM --> ORQ
    ORQ --> PER
    ORQ --> SEM
    ORQ --> NAV
    ORQ --> TTS
    TTS --> UI
    SENS --> PER
    SENS --> MAP
    PER --> SEM
    PER --> MAP
    MAP --> NAV
    SEM --> NAV
    NAV --> SUP
    ORQ --> SUP
    WD --> SUP
    SUP --> ARB --> ADP --> G1
    G1 --> ADP --> SUP
```

## 2. Sensor a mapa, percepcion y memoria

```mermaid
flowchart LR
    CAM[RGB-D] --> SYNC[Sincronizacion]
    LID[LiDAR] --> LIO[FAST-LIO2]
    IMU[IMU] --> LIO
    LIO --> TF[TF map/odom/base]
    LID --> GEO[Mapa geometrico]
    TF --> GEO

    SYNC --> DET[Deteccion]
    SYNC --> DEP[Profundidad]
    DET --> TRK[Tracking]
    DEP --> FUS[Fusion 3D]
    TRK --> FUS
    TF --> FUS

    FUS --> OBS[Observaciones]
    OBS --> ASSOC[Asociacion temporal-espacial]
    ASSOC --> ENT[Entidades]
    ENT --> DB[(PostGIS + pgvector)]
    GEO --> COST[Mapa local/coste]
    ENT --> COST
```

## 3. Instruccion a movimiento G1

```mermaid
sequenceDiagram
    actor U as Usuario
    participant V as Voz/Texto
    participant A as Agente
    participant O as Orquestador
    participant M as Memoria
    participant N as Navegacion
    participant S as Supervisor
    participant G as G1

    U->>V: "Ve a la mesa del laboratorio"
    V->>A: transcripcion + confianza
    A->>O: NavigateTo(entity="mesa laboratorio")
    O->>M: resolver entidad
    M-->>O: region + incertidumbre
    O->>N: NavigationGoal
    loop hasta terminar o cancelar
        N->>S: MotionRequest con vencimiento
        S->>S: limites + obstaculos + salud
        S->>G: SafeMotionCommand
        G-->>S: estado
        N-->>O: progreso
    end
    N-->>O: postcondicion alcanzada
    O-->>A: resultado estructurado
    A-->>V: explicacion
    V-->>U: respuesta
```

## 4. Deteccion, identidad de entidad y persistencia

```mermaid
stateDiagram-v2
    [*] --> Observation
    Observation --> Tentative: primera evidencia
    Tentative --> Confirmed: N observaciones consistentes
    Tentative --> Retired: timeout o inconsistencia
    Confirmed --> Confirmed: nueva asociacion
    Confirmed --> Stale: no observada
    Stale --> Confirmed: reobservacion compatible
    Stale --> Retired: TTL/politica
    Confirmed --> SplitReview: asociacion ambigua
    SplitReview --> Confirmed: evidencia resuelve
    SplitReview --> Retired: entidad descartada
```

Regla: el nombre de una persona nunca se infiere solo por semejanza visual. La
identidad personal requiere consentimiento y una fuente autorizada.

## 5. Seguridad y paro

```mermaid
flowchart TD
    REQ[MotionRequest] --> FRESH{Vigente?}
    FRESH -- no --> ZERO[Orden cero]
    FRESH -- si --> HEALTH{Robot y sensores sanos?}
    HEALTH -- no --> ZERO
    HEALTH -- si --> LOC{Localizacion valida?}
    LOC -- no --> ZERO
    LOC -- degradada --> LIMIT[Reducir velocidad]
    LOC -- valida --> OBS{Espacio libre?}
    OBS -- no --> BRAKE[Frenar]
    OBS -- si --> LIMIT
    LIMIT --> BOUNDS[Limites cinematicos]
    BOUNDS --> CMD[SafeMotionCommand]
    ZERO --> STATE[DEGRADED/ESTOP]
    BRAKE --> STATE
```

## 6. Despliegue fisico

```mermaid
flowchart LR
    subgraph G1["Unitree G1"]
        MCU[Controladores Unitree]
        LID[Mid-360]
        CAM[RGB-D]
        ESTOP[Paro fisico]
    end

    subgraph EDGE["Computadora a bordo / seguridad"]
        DDS[SDK2/DDS]
        SAFE[Supervisor + watchdog]
        LOCAL[Mapa local minimo]
    end

    subgraph AUTO["Computadora de autonomia"]
        LIO[FAST-LIO2 + PGO]
        VISION[Percepcion]
        NAV[Navegacion]
        MEM[Memoria]
        MISSION[Orquestador]
        LOG[MCAP + trazas]
    end

    subgraph OPTIONAL["Servicios opcionales"]
        CLOUD[LLM/ASR/TTS remoto]
        DASH[Panel remoto]
    end

    LID --> LIO
    CAM --> VISION
    LIO --> LOCAL
    LIO --> NAV
    VISION --> NAV
    VISION --> MEM
    MEM --> MISSION
    MISSION --> NAV
    NAV --> SAFE
    LOCAL --> SAFE
    SAFE --> DDS --> MCU
    ESTOP --> MCU
    MISSION -. TLS .-> CLOUD
    LOG -. telemetria .-> DASH
```

## 7. Marcos de coordenadas

```mermaid
flowchart LR
    MAP[map] -->|correccion global| ODOM[odom]
    ODOM -->|LIO continua| BASE[base_link]
    BASE --> LIDAR[lidar_link]
    BASE --> CAMERA[camera_link]
    CAMERA --> COLOR[camera_color_optical_frame]
    CAMERA --> DEPTH[camera_depth_optical_frame]
```

Las transformaciones estaticas se obtienen por calibracion y se versionan. La
pose `map -> odom` puede saltar por cierre de lazo; `odom -> base_link` debe
permanecer continua para control.

## 8. Recuperacion de fallo

```mermaid
sequenceDiagram
    participant H as Health Monitor
    participant O as Orquestador
    participant N as Navegacion
    participant S as Supervisor
    participant R as Robot
    participant L as Registro

    H->>O: localization=LOST
    H->>S: localization=LOST
    S->>R: rampa a cero
    R-->>S: reposo confirmado
    S->>L: evento SAFE_STOP
    O->>N: cancel(goal_id)
    N-->>O: CANCELLED
    O->>L: mision pausada + causa
    O-->>O: intentar relocalizacion limitada
    alt relocalizacion valida
        O->>S: solicitar retorno a SAFE_IDLE
    else presupuesto agotado
        O-->>L: requiere operador
    end
```

## 9. Ciclo de evaluacion

```mermaid
flowchart LR
    SCN[Escenario versionado] --> RUN[Ejecucion]
    RUN --> BAG[MCAP]
    RUN --> TRACE[Trazas]
    BAG --> MET[Metricas]
    TRACE --> MET
    MET --> REP[Reporte]
    REP --> DEC{Supera baseline?}
    DEC -- si --> ADOPT[Adoptar candidato]
    DEC -- no --> KEEP[Mantener baseline]
    ADOPT --> SCN
    KEEP --> SCN
```

Toda sustitucion tecnologica debe recorrer este ciclo con el mismo escenario,
semillas cuando apliquen y configuraciones versionadas.
