export class API {
    constructor (
        private root: string,
    ) {}

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
            fetch(`${this.root}/api${path}`, {
                method: "POST",
                body: body !== null ? JSON.stringify(body) : null,
                headers: { "Content-Type": "application/json" },
            })
                .then(() => resolve())
                .catch(() => reject());
        });
    }

    // API

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
        return this.post("/moveHand", { step })
    }
}