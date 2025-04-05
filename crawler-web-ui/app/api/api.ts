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
            const apiRoot = import.meta.env.VITE_API_ROOT as string;
            fetch(`${apiRoot}/api${path}`, {
                method: "POST",
                body: body !== null ? JSON.stringify(body) : null,
                headers: { "Content-Type": "application/json" },
            })
                .then(() => resolve())
                .catch(() => reject());
        });
    }

    // API

    blink() {
        return this.post("/blink");
    }

    start() {
        return this.post("/start");
    }

    stop() {
        return this.post("/stop");
    }

    moveArm(step: number) {
        return this.post("/moveArm", { step });
    }

    moveHand(step: number) {
        return this.post("/moveHand", { step });
    }
}