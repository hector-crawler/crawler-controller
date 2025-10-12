import classNames from "classnames";
import { useState } from "react";

export function LargeButton({ onClick, children, disabled, smallPadding, extraSmall }: { onClick: () => void, children: React.ReactNode, disabled?: boolean, smallPadding?: boolean, extraSmall?: boolean }) {
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
        { "h-8 font-semibold px-2": extraSmall },
      )}
      onClick={handleClick}
    >
      {children}
    </button>
  );
}

export function InputField({ value, onChange, type, label, disabled }: { value?: any, onChange: (value: string) => void, type?: string, label?: string, disabled?: boolean }) {
  return (
    <div>
      {label && <label className="block text-sm mb-1">{label}</label>}
      <input
        disabled={disabled}
        type={type}
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className="border-2 border-blue-500 rounded-md w-35 p-2 disabled:border-gray-600"
      />
    </div>
  );
}

export function Checkbox({ value, onChange, label, disabled }: { value: boolean, onChange: (value: boolean) => void, label?: string, disabled?: boolean }) {
  return (
    <div className="flex gap-2 items-center mt-1">
      <input
        disabled={disabled}
        type="checkbox"
        checked={value}
        onChange={(e) => onChange(e.target.checked)}
        className="border-2 border-blue-500 rounded-md w-5 h-5 disabled:border-gray-600"
      />
      {label && <label className="block text-sm mb-1 relative translate-y-0.5">{label}</label>}
    </div>
  );
}
