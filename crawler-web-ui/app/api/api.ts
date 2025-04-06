import { useEffect, useState } from "react";
import useWebSocket from "react-use-websocket";

export type ManualState = {
    blinker: boolean,
}

export class API {
    private apiHost = import.meta.env.VITE_API_ROOT || window.location.host

    // HTTP request utilities

    private get(path: string, body: any | null = null) {
        return this.request(path, "GET", body);
    }

    private post(path: string, body: any | null = null) {
        return this.request(path, "POST", body);
    }

    private request(path: string, method: string, body: any | null) {
        return new Promise<void>((resolve, reject) => {
            if (!path.startsWith("/")) path = "/" + path;
            fetch(`http://${this.apiHost}${path}`, {
                method,
                body: body !== null ? JSON.stringify(body) : null,
                headers: { "Content-Type": "application/json" },
            })
                .then(() => resolve())
                .catch(() => reject());
        });
    }

    // API

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

    useManualState() {
        const [state, setState] = useState<ManualState>({ blinker: false });
        const { lastMessage } = useWebSocket(`ws://${this.apiHost}/api/manual/state`);
        useEffect(() => {
            if (lastMessage !== null) setState(JSON.parse(lastMessage.data));
        }, [lastMessage]);
        return state;
    }
}