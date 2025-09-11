import { useEffect, useState } from "react";
import useWebSocket from "react-use-websocket";

export type ManualState = {
    blinker: boolean,
    armPosition: number,
    handPosition: number,
    leftEncoderPosition: number,
    rightEncoderPosition: number,
}

export type MoveMode = "USER_WAIT" | "USER_ARM_UP" | "USER_ARM_DOWN" | "USER_HAND_UP" | "USER_HAND_DOWN" | "USER_STEP" | "USER_STEP_EXPLORATION" | "USER_STEP_EXPLOITATION" | "AUTOMATIC"

export type RLInternals = {
    rlEnvironmentInternals: null | {
        loopState: 0 | 1 | 2,
        latestStateReward: {
            armPosition: number,
            handPosition: number,
            reward: number,
        },
        latestAction: {
            moveArm: number,
            moveHand: number,
        },
        progress: number[],
    },
    qLearning: null | {
        armStates: number,
        handStates: number,
        armStep: number,
        handStep: number,
        learningRate: number,
        explorationRate: number,
        explorationDecayFactor: number,
        minExplorationRate: number,
        discountFactor: number,
        qTableCols: string[],
        qTableRows: string[],
        qTableValues: number[],
        moveIsExploration: boolean,
        moveMode: MoveMode,
        waitingForUserMove: boolean,
    }
}

export type QLearningConfiguration = {
    armStates: number,
    handStates: number,
    armStep: number,
    handStep: number,
    learningRate: number,
    explorationRate: number,
    explorationDecayFactor: number,
    minExplorationRate: number,
    discountFactor: number,
    initialMoveModeWait: boolean,
}

export class API {
    private apiHost = import.meta.env.VITE_API_ROOT || window.location.host
    private apiOrigin = import.meta.env.VITE_API_ROOT !== undefined ? `http://${this.apiHost}` : window.location.origin
    public readonly isDev = import.meta.env.VITE_API_ROOT?.includes("localhost")

    // HTTP request utilities

    private get(path: string, body: any | null = null) {
        return this.request(path, "GET", body);
    }

    private post(path: string, body: any | null = null) {
        return this.request(path, "POST", body);
    }

    private request(path: string, method: string, body: any | null) {
        return new Promise<string>((resolve, reject) => {
            if (!path.startsWith("/")) path = "/" + path;
            fetch(`${this.apiOrigin}${path}`, {
                method,
                body: body !== null ? JSON.stringify(body) : null,
                headers: { "Content-Type": "application/json" },
            })
                .then(response => response.text().then(text => resolve(text)))
                .catch(() => reject());
        });
    }

    private ws<T>(path: string, defaultValue: T) {
        if (!path.startsWith("/")) path = "/" + path;
        const [state, setState] = useState<T>(defaultValue);
        const wsProtocol = this.apiOrigin.startsWith("https") ? "wss" : "ws";
        const { lastMessage } = useWebSocket(`${wsProtocol}://${this.apiHost}${path}`)
        useEffect(() => {
            if (lastMessage !== null) setState(JSON.parse(lastMessage.data));
        }, [lastMessage]);
        return state;
    }

    // API

    getBuildMetadata() {
        return this.get("/buildMetadata");
    }

    toggleBlinker() {
        return this.post("/api/manual/blinker/toggle");
    }

    writeBlinker(state: boolean) {
        return this.post("/api/manual/blinker/write", { state });
    }

    moveArm(step: number) {
        return this.post("/api/manual/moveArm", { step });
    }

    moveHand(step: number) {
        return this.post("/api/manual/moveHand", { step });
    }

    mockLeftEncoder(position: number) {
        return this.post("/api/manual/mockLeftEncoder", { position });
    }

    mockRightEncoder(position: number) {
        return this.post("/api/manual/mockRightEncoder", { position });
    }

    useManualState() {
        return this.ws("/api/manual/state", { blinker: false, armPosition: NaN, handPosition: NaN, leftEncoderPosition: NaN, rightEncoderPosition: NaN } as ManualState);
    }

    startRLQLearning(configuration: QLearningConfiguration) {
        return this.post("/api/rl/start/qLearning", configuration);
    }

    setQLearningMoveMode(moveMode: MoveMode) {
        return this.post("/api/rl/qLearning/setMoveMode", { moveMode });
    }

    stopRL() {
        return this.post("/api/rl/stop");
    }

    useRLInternals(): RLInternals {
        return this.ws("/api/rl/internals", { rlEnvironmentInternals: null, qLearning: null });
    }
}
