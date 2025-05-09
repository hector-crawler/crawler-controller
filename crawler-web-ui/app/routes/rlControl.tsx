import { LargeButton } from "~/components";
import type { Route } from "./+types/home";
import { API } from "~/api/api";

export function meta({ }: Route.MetaArgs) {
    return [
        { title: "RL Control | Crawler" },
    ];
}

export default function RLControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
    const api = new API();

    return (
        <main>
            
            {/* start and stop buttons */}
            <div className="flex flex-col items-center p-10 gap-4">
                <LargeButton onClick={async () => await api.startRL()}>Start</LargeButton>
                <LargeButton onClick={async () => await api.stopRL()}>Stop</LargeButton>
            </div>

        </main>
    )
}