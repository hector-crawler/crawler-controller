import { InputField, LargeButton } from "~/components";
import type { Route } from "./+types/home";
import { API, type QLearningConfiguration } from "~/api/api";
import { useState } from "react";
import { interpolateColor } from "~/ui/util";
import classNames from "classnames";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "RL Control | Crawler" },
  ];
}

export default function RLControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
  const api = new API();

  return (
    <main>
      <div className="flex flex-col items-center p-10 gap-4">

        <QLearningControl api={api} />

      </div>
    </main>
  )
}

function QLearningControl({ api }: { api: API }) {
  const rlInternals = api.useRLInternals();

  const [configuration, setConfiguration] = useState<QLearningConfiguration>({
    armStates: 0,
    handStates: 0,
    armStep: 0,
    handStep: 0,
    learningRate: 0,
    explorationRate: 0,
    explorationDecayRate: 0,
    maxExplorationRate: 0,
    minExplorationRate: 0,
    discountFactor: 0,
  })
  const start = async () => {
    await api.startRLQLearning(configuration);
  }
  const stop = async () => {
    await api.stopRL();
  }

  return (
    <div className="flex flex-col items-center p-4 gap-4 border-blue-500 border-1 rounded-xl">
      <div className="p-0">Q-learning {rlInternals.qLearning ? <span className="text-green-500">(active)</span> : <span className="text-gray-500">(inactive)</span>}</div>

      {rlInternals.qLearning === null && (() => {
        return <>
          {/* configure parameters */}
          <div className="flex flex-col gap-2">
            <InputField type="number" label="arm states" value={configuration.armStates} onChange={str => setConfiguration(config => ({ ...config, armStates: Number(str) }))} />
            <InputField type="number" label="hand states" value={configuration.handStates} onChange={str => setConfiguration(config => ({ ...config, handStates: Number(str) }))} />
            <InputField type="number" label="arm step" value={configuration.armStep} onChange={str => setConfiguration(config => ({ ...config, armStep: Number(str) }))} />
            <InputField type="number" label="hand step" value={configuration.handStep} onChange={str => setConfiguration(config => ({ ...config, handStep: Number(str) }))} />
            <InputField type="number" label="learning rate" value={configuration.learningRate} onChange={str => setConfiguration(config => ({ ...config, learningRate: Number(str) }))} />
            <InputField type="number" label="exploration rate" value={configuration.explorationRate} onChange={str => setConfiguration(config => ({ ...config, explorationRate: Number(str) }))} />
            <InputField type="number" label="exploration decay rate" value={configuration.explorationDecayRate} onChange={str => setConfiguration(config => ({ ...config, explorationDecayRate: Number(str) }))} />
            <InputField type="number" label="max exploration rate" value={configuration.maxExplorationRate} onChange={str => setConfiguration(config => ({ ...config, maxExplorationRate: Number(str) }))} />
            <InputField type="number" label="min exploration rate" value={configuration.minExplorationRate} onChange={str => setConfiguration(config => ({ ...config, minExplorationRate: Number(str) }))} />
            <InputField type="number" label="discount factor" value={configuration.discountFactor} onChange={str => setConfiguration(config => ({ ...config, discountFactor: Number(str) }))} />
          </div>

          <div className="width-full flex justify-end">
            <LargeButton onClick={start} smallPadding={true}>Start</LargeButton>
          </div>
        </>
      })()}

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
            <div>
              Parameters:<br />
              {Object.entries({
                "arm states": rlInternals.qLearning.armStates,
                "hand states": rlInternals.qLearning.handStates,
                "arm step": rlInternals.qLearning.armStep,
                "hand step": rlInternals.qLearning.handStep,
                "learning rate": rlInternals.qLearning.learningRate,
                "exploration rate": rlInternals.qLearning.explorationRate,
                "exploration decay rate": rlInternals.qLearning.explorationDecayRate,
                "discount factor": rlInternals.qLearning.discountFactor,
              }).map(([key, value]) => (
                <div key={key} className="ml-2">
                  <span className="text-gray-400">{key}: </span>
                  <span>{value}</span>
                </div>
              ))}
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
    return <div key={key}
      className={classNames("flex justify-center items-center m-0 px-1.5 py-0.5 rounded-md", {"font-mono border-1 border-blue-800": isQValue})}
      style={{backgroundColor: interpolateColor("#030712" /* gray-800 */, "#3b82f6" /* blue-500 */, value)}}
    >{text}</div>;
  }

  const cols = columnLabels.length;
  const valueRows: number[][] = [];
  for (let i = 0; i < values.length; i += cols) {
    valueRows.push(values.slice(i, i + cols));
  }

  const bodyCells = []
  for (let i = 0; i < rowLabels.length; i++) {
    const averageRowValue = valueRows[i].reduce((acc, value) => acc + value, 0) / valueRows[i].length;
    bodyCells.push(
      // row label
      cell(`row-label-${i}`, rowLabels[i], false, averageRowValue)
    )
    bodyCells.push(
      // value cells
      ...(valueRows[i].map((value, j) => {
        const relativeQValue = value; //(value - minQValue) / (maxQValue - minQValue);
        return cell(`${i}`, value.toFixed(3), true, relativeQValue);
      }))
    )
  }

  return (
    <div className="grid gap-2" style={{gridTemplateColumns: `repeat(${columnLabels.length + 1}, auto)`}}>
      <div></div>
      {columnLabels.map((col, i) => {
        // column label
        const averageColValue = valueRows.map(row => row[i]).reduce((acc, value) => acc + value, 0) / valueRows.length;
        return cell(`col-label-${i}`, col, false, averageColValue);
      })}
      {bodyCells}
    </div>
  );
}