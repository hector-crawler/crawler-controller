import type { Route } from "./+types/home";
import { API } from "../api/api";
import classNames from "classnames";
import { useState } from "react";
import { DownIcon, UpIcon } from "~/ui/icons";

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
        <MotorController onMove={async (factor) => await api.moveArm(factor * 10)} position={manualState.armPosition} />
        <MotorController onMove={async (factor) => await api.moveHand(factor * 10)} position={manualState.handPosition} />
      </div>
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

function MotorController({ onMove, position }: { onMove: (factor: number) => void, position: number }) {
  if (isNaN(position)) position = 0;
  return (
    <div className="flex flex-col justify-center gap-3 border-blue-500 border-1 p-2 rounded-xl">
      <LargeButton onClick={() => onMove(1)} smallPadding={true}><UpIcon /></LargeButton>
      <div className="flex justify-center items-center h-30 relative">
        <div className="absolute left-50% bg-gray-700 w-1.5 h-30 rounded-full"></div>
        <div className="absolute bg-gray-800 w-10 h-6 rounded-xl text-sm flex justify-center items-center top-0 transition-[top]" style={{ top: (100 - (isNaN(position) ? 0 : position)) / 100 * (26 * 4) }}>{position}</div>
      </div>
      <LargeButton onClick={() => onMove(-1)} smallPadding={true}><DownIcon /></LargeButton>
    </div >
  )
}

function LargeButton({ onClick, children, disabled, smallPadding }: { onClick: () => void, children: React.ReactNode, disabled?: boolean, smallPadding?: boolean }) {
  const [isLit, setIsLit] = useState(false);

  const handleClick = () => {
    onClick();
    setIsLit(true);
    setTimeout(() => {
      setIsLit(false);
    }, 500);
  };

  return (
    <button
      disabled={disabled}
      className={classNames(
        "flex justify-center items-center",
        "bg-blue-600 rounded-xl uppercase font-bold cursor-pointer",
        "disabled:bg-gray-600 disabled:cursor-auto",
        {
          "bg-orange-500": isLit,
          "bg-blue-600": !isLit,
          "transition-[background] ease-out duration-500": !isLit,
        },
        { "p-3": !smallPadding, "p-2": smallPadding },
      )}
      onClick={handleClick}
    >
      {children}
    </button>
  );
}