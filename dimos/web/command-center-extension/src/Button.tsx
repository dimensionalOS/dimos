interface ButtonProps {
  onClick: () => void;
  isActive: boolean;
  children: React.ReactNode;
}

export default function Button({ onClick, isActive, children }: ButtonProps): React.ReactElement {
  return (
    <button
      onClick={onClick}
      style={{
        backgroundColor: isActive ? "#ff4444" : "#4CAF50",
        color: "white",
        padding: "5px 10px",
        border: "none",
        borderRadius: "4px",
        cursor: "pointer",
        fontSize: "14px",
      }}
    >
      {children}
    </button>
  );
}
