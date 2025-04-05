import type { Route } from "./+types/home";
import { API } from "../api/api";
import classNames from "classnames";
import { useState } from "react";

export function meta({ }: Route.MetaArgs) {
  return [
    { title: "Manual Control | Crawler" },
  ];
}

export default function ManualControl({ loaderData }: { loaderData: Route.LoaderArgs }) {
  const api = new API();

  const [available, setAvailable] = useState(false);

  return (
    <main className="flex flex-col items-center p-10 gap-6">
      <div className="flex justify-center">
        <LargeButton label="blink" onClick={async () => await api.blink()} />
      </div>

      <div className="flex justify-center">
        <LargeButton label={available ? "stop" : "start"} onClick={async () => {
          if (!available) {
            await api.start();
            setAvailable(true);
          } else {
            await api.stop();
            setAvailable(false);
          }
        }} />
      </div>

      <div className="grid grid-cols-2 gap-4">
        <LargeButton onClick={() => api.moveHand(100)} label="hand up" square={true} disabled={!available} />
        <LargeButton onClick={() => api.moveArm(100)} label="arm up" square={true} disabled={!available} />
        <LargeButton onClick={() => api.moveHand(-100)} label="hand down" square={true} disabled={!available} />
        <LargeButton onClick={() => api.moveArm(-100)} label="arm down" square={true} disabled={!available} />
      </div>
    </main>
  );
}

function LargeButton({
  onClick,
  label,
  square,
  disabled,
}: {
  onClick: () => void;
  label: string;
  square?: boolean;
  disabled?: boolean;
}) {
  const [isLit, setIsLit] = useState(false);

  const handleClick = () => {
    if (isLit) return;
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
        "p-3 bg-blue-600 rounded-xl uppercase font-bold cursor-pointer",
        "disabled:bg-gray-600 disabled:cursor-auto",
        {
          "w-40 h-40": square,
          "bg-orange-500": isLit,
          "bg-blue-600": !isLit,
          "transition-[background] ease-out duration-500": !isLit,
        }
      )}
      onClick={handleClick}
    >
      {label}
    </button>
  );
}