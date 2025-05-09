import classNames from "classnames";
import { useState } from "react";

export function LargeButton({ onClick, children, disabled, smallPadding }: { onClick: () => void, children: React.ReactNode, disabled?: boolean, smallPadding?: boolean }) {
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
  