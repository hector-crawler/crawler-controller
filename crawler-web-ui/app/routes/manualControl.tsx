import type { Route } from "./+types/home";
import { API } from "../api/api";

export function meta({}: Route.MetaArgs) {
  return [
    { title: "Manual Control | Crawler" },
  ];
}

export default function ManualControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
    const api = new API(""); //TODO

    return (
        <main className="flex items-center justify-center">
            <h1>Manual Control</h1>

            <button onClick={() => api.start()}>start</button>
            <button onClick={() => api.stop()}>stop</button>

            <button onClick={() => api.moveArm(100)}>up</button>
            <button onClick={() => api.moveArm(-100)}>down</button>
        </main>
    );
}