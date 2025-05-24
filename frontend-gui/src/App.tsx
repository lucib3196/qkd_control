import { useEffect, useState } from "react";
import api from "../api";

export type ArucoMsg = {
  pose: {
    position: {
      x: number;
      y: number;
      z: number;
    };
    orientation: {
      x: number;
      y: number;
      z: number;
      w: number;
    };
  };
  detection_time: {
    sec: number;
    nanosec: number;
  };
  size: number;
  dictionary_type: string; // e.g., "DICT_4X4_50"
};

const ArucoSection: React.FC = () => {
  const [ArucoData, setArucoData] = useState<ArucoMsg | null>(null);

  const getArucoData = async () => {
    try {
      const res = await api.get<ArucoMsg>("http://localhost:8000/arucodata");
      setArucoData(res.data);
    } catch (err) {
      console.error("Failed to fetch Aruco data:", err);
    }
  };

  useEffect(() => {
    getArucoData();
  }, []);

  return (
    <div>
      {ArucoData?.pose ? JSON.stringify(ArucoData.pose) : "No pose data"}
    </div>
  );
};

function App() {
  return (
    <>
      <div className="border border-amber-500">Hello</div>
      <ArucoSection />
    </>
  );
}

export default App;
