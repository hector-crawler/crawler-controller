import { useEffect, useState } from "react";
import useWebSocket from "react-use-websocket";

export class API {
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
            fetch(`http://${window.location.host}${path}`, {
                method,
                body: body !== null ? JSON.stringify(body) : null,
                headers: { "Content-Type": "application/json" },
            })
                .then(() => resolve())
                .catch(() => reject());
        });
    }

    // API

    blink() {
        return this.post("/api/blinker/toggle");
    }

    start() {
        return this.post("/api/start");
    }

    stop() {
        return this.post("/api/stop");
    }

    moveArm(step: number) {
        return this.post("/api/moveArm", { step });
    }

    moveHand(step: number) {
        return this.post("/api/moveHand", { step });
    }

    useBlinkerState() {
        const [state, setState] = useState<boolean | null>(null);
        const { lastMessage } = useWebSocket(`ws://${window.location.host}/state/blinker/state`);
        useEffect(() => {
            const data = lastMessage !== null ? JSON.parse(lastMessage!.data) : { state: false };
            setState(data.state);
        }, [lastMessage]);
        return state;
    }
}