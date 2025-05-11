import { InputField, LargeButton } from "~/components";
import type { Route } from "./+types/home";
import { API } from "~/api/api";
import { useState } from "react";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "RL Control | Crawler" },
  ];
}

export default function RLControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
  const api = new API();

  const [paramA, setParamA] = useState(0);
  const [paramB, setParamB] = useState(0);
  const [isRunning, setIsRunning] = useState(false);

  const start = async () => {
    await api.startRLQLearning(paramA, paramB);
    setIsRunning(true);
  }
  const stop = async () => {
    await api.stopRL();
    setIsRunning(false);
  }

  return (
    <main>
      <div className="flex flex-col items-center p-10 gap-4">

        {/* configure q_learning */}
        <div className="flex flex-col items-center p-4 gap-4 border-blue-500 border-1 rounded-xl">
          <div className="p-0">Configure Q-learning</div>

          {/* configure parameters */}
          <div className="flex flex-col gap-2">
            <InputField disabled={isRunning} type="number" label="paramA" value={paramA} onChange={str => setParamA(Number(str))} />
            <InputField disabled={isRunning} type="number" label="paramB" value={paramB} onChange={str => setParamB(Number(str))} />
          </div>

          {/* start and stop buttons */}
          <div className="flex gap-4">
            <LargeButton disabled={isRunning} onClick={start}>Start</LargeButton>
            <LargeButton disabled={!isRunning} onClick={stop}>Stop</LargeButton>
          </div>

        </div>

        {/* q_learning state */}
        <div className="flex flex-col items-center p-4 gap-4 border-blue-500 border-1 rounded-xl">
          <div className="p-0">Q-learning state</div>

          <span className="text-gray-500">( internal state of the RL algorithm )</span>
        </div>

      </div>
    </main>
  )
}