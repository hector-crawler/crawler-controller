import type { Route } from "./+types/home";
import { API } from "../api/api";
import classNames from "classnames";
import { DownIcon, UpIcon } from "~/ui/icons";
import { LargeButton } from "~/components";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "Manual Control | Crawler" },
  ];
}

export default function ManualControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
  const api = new API();

  const manualState = api.useManualState();

  return (
    <main className="flex flex-col items-center p-10 gap-6">

      {/* blinker */}
      <BlinkerController
        onToggle={async () => await api.toggleBlinker()}
        onOn={async () => await api.writeBlinker(true)}
        onOff={async () => await api.writeBlinker(false)}
        state={manualState.blinker}
      />

      {/* motors */}
      <div className="flex gap-6">
        {/* todo: get motor limits from actual source of truth */}
        <MotorController onMove={async (factor) => await api.moveArm(factor * 100)} minPosition={900} maxPosition={1500} position={manualState.armPosition} />
        <MotorController onMove={async (factor) => await api.moveHand(factor * 100)} minPosition={2900} maxPosition={3700} position={manualState.handPosition} />
      </div>

      {/* encoders */}
      <div className="flex gap-6">
        <EncodersController position={manualState.leftEncoderPosition} />
        <EncodersController position={manualState.rightEncoderPosition} />
      </div>

      {api.isDev && <div className="grid grid-cols-2 gap-3">
        <LargeButton smallPadding={true} onClick={async () => await api.mockLeftEncoder(manualState.leftEncoderPosition + 10)}>left +10</LargeButton>
        <LargeButton smallPadding={true} onClick={async () => await api.mockRightEncoder(manualState.rightEncoderPosition + 10)}>right +10</LargeButton>
        <LargeButton smallPadding={true} onClick={async () => await api.mockLeftEncoder(manualState.leftEncoderPosition - 10)}>left -10</LargeButton>
        <LargeButton smallPadding={true} onClick={async () => await api.mockRightEncoder(manualState.rightEncoderPosition - 10)}>right -10</LargeButton>
      </div>}
    </main>
  );
}

function BlinkerController({ onToggle, onOn, onOff, state }: { onToggle: () => void, onOn: () => void, onOff: () => void, state: boolean }) {
  return (
    <div className="flex justify-center gap-2 border-blue-500 border-1 p-2 rounded-xl">
      <div className="flex justify-center items-center pl-1">
        <div className={classNames("h-8 w-8 rounded-full bg-gray-700 transition-[background] duration-100", { "bg-white": state })}></div>
      </div>
      <LargeButton onClick={onToggle}>toggle</LargeButton>
      <LargeButton onClick={onOn}>on</LargeButton>
      <LargeButton onClick={onOff}>off</LargeButton>
    </div>
  )
}

function MotorController({ onMove, position, minPosition, maxPosition }: { onMove: (factor: number) => void, position: number, minPosition: number, maxPosition: number }) {
  const displayPosition =
    isNaN(position) ? (minPosition + (maxPosition - minPosition) / 2)
      : position > maxPosition ? maxPosition
        : position < minPosition ? minPosition
          : position
  const positionPercentage = (displayPosition - minPosition) / (maxPosition - minPosition)

  return (
    <div className="flex flex-col justify-center gap-3 border-blue-500 border-1 p-2 rounded-xl">
      <LargeButton onClick={() => onMove(1)} smallPadding={true}><UpIcon /></LargeButton>
      <div className="flex justify-center items-center h-30 relative">
        <div className="absolute left-50% bg-gray-700 w-1.5 h-30 rounded-full"></div>
        <div className="absolute bg-gray-800 w-10 h-6 rounded-xl text-sm flex justify-center items-center top-0 transition-[top]" style={{ top: (1 - positionPercentage) * (26 * 4) }}>{!isNaN(position) ? position : ""}</div>
      </div>
      <LargeButton onClick={() => onMove(-1)} smallPadding={true}><DownIcon /></LargeButton>
    </div >
  )
}

function EncodersController({ position }: { position: number }) {
  return (
    <div className="rounded-full border-blue-500 border-1 size-25 flex justify-center items-center relative">
      <div className="absolute size-full flex justify-center items-center">
        <div className="bg-gray-800 p-0.5 w-10 h-6 rounded-xl text-sm flex justify-center items-center">{!isNaN(position) ? position : ""}</div>
      </div>
      <div className="absolute size-19 border-gray-700 border-5 rounded-full transition-[transform]" style={{ transform: `rotate(${position}deg)` }}>
        <div className="absolute translate-x-[calc(var(--spacing)*7.3)] -translate-y-2 w-2 h-4 bg-blue-500 rounded-xl"></div>
      </div>
    </div>
  )
}
