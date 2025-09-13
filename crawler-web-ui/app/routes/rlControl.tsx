import { Checkbox, InputField, LargeButton } from "~/components";
import type { Route } from "./+types/home";
import { API, type QLearningConfiguration, type RLInternals } from "~/api/api";
import { useEffect, useState } from "react";
import { interpolateColor } from "~/ui/util";
import classNames from "classnames";
import { Line } from "react-chartjs-2";
import { CategoryScale, Chart, Legend, LinearScale, LineElement, PointElement, Title, Tooltip } from "chart.js";
import { useLocalStorage } from "@uidotdev/usehooks";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "RL Control | Crawler" },
  ];
}

export default function RLControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
  const api = new API();
  const rlInternals = api.useRLInternals();

  return (
    <main>
      <div className="flex p-10 gap-4 items-start">

        <RLEnvironmentControl api={api} rlInternals={rlInternals} />

        <QLearningControl api={api} rlInternals={rlInternals} />

      </div>
    </main>
  )
}

function RLEnvironmentControl({ api, rlInternals }: { api: API, rlInternals: RLInternals }) {
  if (rlInternals.rlEnvironmentInternals === null) return <></>;
  const internals = rlInternals.rlEnvironmentInternals;

  const [onlyLatestProgress, setOnlyLatestProgress] = useState(false);
  const MAX_PROGRESS_ENTRIES = 50;
  const progressStartIndex = (onlyLatestProgress && rlInternals.rlEnvironmentInternals.progress.length > MAX_PROGRESS_ENTRIES) ? rlInternals.rlEnvironmentInternals.progress.length - MAX_PROGRESS_ENTRIES : 0;
  const progress = rlInternals.rlEnvironmentInternals.progress.slice(progressStartIndex);
  Chart.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);

  return (
    <div className="flex flex-col items-center p-4 gap-4 border-blue-500 border-1 rounded-xl">
      <div className="p-0">RL {internals.loopState === 0 ? <span className="text-gray-500">(stopped)</span> : internals.loopState === 1 ? <span className="text-yellow-500">(waiting)</span> : <span className="text-blue-500">(acting)</span>}</div>

      {internals.loopState !== 0 && (
        <div className="width-full flex flex-col gap-4">
          <div className="flex items-center gap-2">
            Reward:
            <div className={classNames("px-2 py-1 rounded-lg font-mono font-bold", { "bg-gray-600": internals.latestStateReward.reward === 0, "bg-green-700": internals.latestStateReward.reward > 0, "bg-red-700": internals.latestStateReward.reward < 0 })}>
              {internals.latestStateReward.reward > 0 ? `+${internals.latestStateReward.reward}` : internals.latestStateReward.reward}
            </div>
          </div>

          <div className="flex items-center gap-2">
            Latest action:
            <div className="ml-2 flex gap-1.5 items-center">
              <div className="px-2 py-1 rounded-lg font-mono font-bold bg-gray-800 flex gap-0.5 items-center">
                <span className="text-sm translate-y-[1px]">{internals.latestAction.moveArm > 0 ? "🡩" : internals.latestAction.moveArm < 0 ? "🡫" : "🡪"}</span>
                <span>{Math.abs(internals.latestAction.moveArm)}</span>
              </div>
              /
              <div className="px-2 py-1 rounded-lg font-mono font-bold bg-gray-800 flex gap-0.5 items-center">
                <span className="text-sm translate-y-[1px]">{internals.latestAction.moveHand > 0 ? "🡩" : internals.latestAction.moveHand < 0 ? "🡫" : "🡪"}</span>
                <span>{Math.abs(internals.latestAction.moveHand)}</span>
              </div>
            </div>
          </div>

          <Line options={{
            responsive: true,
            animation: false,
          }} data={{
            labels: progress.map((_, i) => progressStartIndex + i),
            datasets: [{
              label: "Progress",
              data: progress,
              borderColor: "#3b82f6",
              pointStyle: false,
            }]
          }}></Line>
          <div className="w-full flex justify-end">
            <div className="flex gap-1 cursor-pointer">
              <input type="checkbox" id="show-only-latest" className="bg-blue-600 rounded-md" onClick={() => setOnlyLatestProgress(!onlyLatestProgress)}></input>
              <label htmlFor="show-only-latest" className="text-sm text-gray-400">{onlyLatestProgress ? "show all" : "show only latest"}</label>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

const defaultQLearningConfiguration: QLearningConfiguration = {
  armStates: 3,
  handStates: 3,
  armStep: 200,
  handStep: 200,
  learningRate: 0.5,
  explorationRate: 1.0,
  explorationDecayFactor: 0.95,
  minExplorationRate: 0.01,
  discountFactor: 0.99,
  initialMoveModeWait: false,
};

function QLearningControl({ api, rlInternals }: { api: API, rlInternals: RLInternals }) {
  const [configuration, setConfiguration] = useLocalStorage("qLearningConfiguration", defaultQLearningConfiguration);
  const start = async () => {
    await api.startRLQLearning(configuration);
  }
  const stop = async () => {
    await api.stopRL();
  }

  return (
    <div className="flex flex-col items-center p-4 gap-4 border-blue-500 border-1 rounded-xl">
      <div className="p-0">Q-learning {rlInternals.qLearning ? <span className="text-green-500">(active)</span> : <span className="text-gray-500">(inactive)</span>}</div>

      {/* (inactive) */}
      {rlInternals.qLearning === null && <>
        {/* configure parameters */}
        <div className="flex flex-col gap-2">
          <div className="flex gap-2">
            <InputField type="number" label="arm states" value={configuration.armStates} onChange={str => setConfiguration(config => ({ ...config, armStates: Number(str) }))} />
            <InputField type="number" label="hand states" value={configuration.handStates} onChange={str => setConfiguration(config => ({ ...config, handStates: Number(str) }))} />
          </div>
          <div className="flex gap-2">
            <InputField type="number" label="arm step" value={configuration.armStep} onChange={str => setConfiguration(config => ({ ...config, armStep: Number(str) }))} />
            <InputField type="number" label="hand step" value={configuration.handStep} onChange={str => setConfiguration(config => ({ ...config, handStep: Number(str) }))} />
          </div>
          <InputField type="number" label="learning rate" value={configuration.learningRate} onChange={str => setConfiguration(config => ({ ...config, learningRate: Number(str) }))} />
          <div className="flex gap-2">
            <InputField type="number" label="exploration rate" value={configuration.explorationRate} onChange={str => setConfiguration(config => ({ ...config, explorationRate: Number(str) }))} />
            <InputField type="number" label="exploration decay factor" value={configuration.explorationDecayFactor} onChange={str => setConfiguration(config => ({ ...config, explorationDecayFactor: Number(str) }))} />
          </div>
          <div className="flex gap-2">
            <InputField type="number" label="min exploration rate" value={configuration.minExplorationRate} onChange={str => setConfiguration(config => ({ ...config, minExplorationRate: Number(str) }))} />
          </div>
          <InputField type="number" label="discount factor" value={configuration.discountFactor} onChange={str => setConfiguration(config => ({ ...config, discountFactor: Number(str) }))} />
          <Checkbox label="initial move mode: wait for user" value={configuration.initialMoveModeWait} onChange={checked => setConfiguration(config => ({ ...config, initialMoveModeWait: checked }))} />
          <div className="flex justify-between mt-2">
            <LargeButton smallPadding={true} disabled={JSON.stringify(configuration) === JSON.stringify(defaultQLearningConfiguration)} onClick={() => setConfiguration(defaultQLearningConfiguration)}>Reset to default</LargeButton>
            <LargeButton onClick={start} smallPadding={true}>Start</LargeButton>
          </div>
        </div>
      </>}

      {/* (active) */}
      {rlInternals.qLearning && (
        <div className="flex gap-5 flex-col">
          <div className="flex gap-5">
            {/* display Q-table */}
            <HeatmapTable
              columnLabels={rlInternals.qLearning.qTableCols}
              rowLabels={rlInternals.qLearning.qTableRows}
              values={rlInternals.qLearning.qTableValues}
            />

            {/* display parameters */}
            <div className="flex flex-col gap-3">
              <LabeledValues label="Parameters" table={{
                "arm states": rlInternals.qLearning.armStates.toString(),
                "hand states": rlInternals.qLearning.handStates.toString(),
                "arm step": rlInternals.qLearning.armStep.toString(),
                "hand step": rlInternals.qLearning.handStep.toString(),
                "learning rate": rlInternals.qLearning.learningRate.toString(),
                "exploration rate": rlInternals.qLearning.explorationRate.toFixed(6),
                "exploration decay factor": rlInternals.qLearning.explorationDecayFactor.toString(),
                "discount factor": rlInternals.qLearning.discountFactor.toString(),
              }} />
              {(rlInternals.qLearning.moveIsExploration
                ? <div className="flex gap-2 items-center"><div className="border-blue-500 border-3 font-bold size-7 flex justify-center items-center rounded-full">?</div> Exploration</div>
                : <div className="flex gap-2 items-center"><div className="border-blue-500 border-3 font-bold size-7 flex justify-center items-center rounded-sm">&gt;</div> Exploitation</div>
              )}

              {/* move mode */}
              <div className="flex flex-wrap gap-2">
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_WAIT")} disabled={rlInternals.qLearning.moveMode === "USER_WAIT"}>wait</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_ARM_UP")} disabled={rlInternals.qLearning.moveMode === "USER_ARM_UP"}>arm up</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_ARM_DOWN")} disabled={rlInternals.qLearning.moveMode === "USER_ARM_DOWN"}>arm down</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_HAND_UP")} disabled={rlInternals.qLearning.moveMode === "USER_HAND_UP"}>hand up</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_HAND_DOWN")} disabled={rlInternals.qLearning.moveMode === "USER_HAND_DOWN"}>hand down</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_STEP")} disabled={rlInternals.qLearning.moveMode === "USER_STEP"}>step</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_STEP_EXPLORATION")} disabled={rlInternals.qLearning.moveMode === "USER_STEP_EXPLORATION"}>step (explore)</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("USER_STEP_EXPLOITATION")} disabled={rlInternals.qLearning.moveMode === "USER_STEP_EXPLOITATION"}>step (exploit)</LargeButton>
                <LargeButton smallPadding={true} onClick={() => api.setQLearningMoveMode("AUTOMATIC")} disabled={rlInternals.qLearning.moveMode === "AUTOMATIC"}>automatic</LargeButton>
                {rlInternals.qLearning.waitingForUserMove && <div className="bg-red-500 animate-[pulse_700ms_infinite] size-5 rounded-full"></div>}
              </div>
            </div>
          </div>

          <div className="width-full flex justify-end">
            <LargeButton onClick={stop} smallPadding={true}>Stop</LargeButton>
          </div>
        </div>
      )}
    </div>
  )
}

function HeatmapTable({ columnLabels, rowLabels, values }: { columnLabels: string[], rowLabels: string[], values: number[] }) {
  const cell = (key: string, text: string, isQValue: boolean, value: number) => {
    const [highlighted, setHighlighted] = useState(false);
    const [lastValue, setLastValue] = useState(value);
    if (isQValue) {
      useEffect(() => {
        if (value !== lastValue) {
          setLastValue(value);
          setHighlighted(true);
          const timeout = setTimeout(() => setHighlighted(false), 750);
          return () => clearTimeout(timeout);
        }
      }, [value])
    }
    return <div key={key}
      className={classNames("flex justify-center items-center m-0 px-1.5 py-0.5 rounded-md transition-[background-color,outline-color] outline-2 outline-transparent", { "font-mono border-1 border-blue-800": isQValue }, { "outline-white!": highlighted })}
      style={{ backgroundColor: interpolateColor("#030712" /* gray-800 */, "#3b82f6" /* blue-500 */, value) }}
    >{text}</div>;
  }

  const cols = columnLabels.length;
  const valueRows: number[][] = [];
  for (let i = 0; i < values.length; i += cols) {
    valueRows.push(values.slice(i, i + cols));
  }

  const minQValue = Math.min(...values);
  const maxQValue = Math.max(...values);
  const relativizeQValue = (value: number) => (value - minQValue) / (maxQValue - minQValue);

  const bodyCells = []
  for (let i = 0; i < rowLabels.length; i++) {
    const averageRowValue = valueRows[i].reduce((acc, value) => acc + value, 0) / valueRows[i].length;
    bodyCells.push(
      // row label
      cell(`row-label-${i}`, rowLabels[i], false, relativizeQValue(averageRowValue))
    )
    bodyCells.push(
      // value cells
      ...(valueRows[i].map((value, j) => {
        return cell(`${i}`, value.toFixed(3), true, relativizeQValue(value));
      }))
    )
  }

  return (
    <div className="grid gap-2" style={{ gridTemplateColumns: `repeat(${columnLabels.length + 1}, auto)` }}>
      <div></div>
      {columnLabels.map((col, i) => {
        // column label
        const averageColValue = valueRows.map(row => row[i]).reduce((acc, value) => acc + value, 0) / valueRows.length;
        return cell(`col-label-${i}`, col, false, relativizeQValue(averageColValue));
      })}
      {bodyCells}
    </div>
  );
}

function LabeledValues({ label, table }: { label: string, table: { [key: string]: string } }) {
  return (
    <div>
      {label}:<br />
      {Object.entries(table).map(([key, value]) => (
        <div key={key} className="ml-2">
          <span className="text-gray-400">{key}: </span>
          <span>{value}</span>
        </div>
      ))}
    </div>
  );
}
