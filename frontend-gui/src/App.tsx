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

const mockArucoData: ArucoMsg = {
  pose: {
    position: {
      x: 1.25,
      y: -0.75,
      z: 3.5,
    },
    orientation: {
      x: 0.02,
      y: -0.1,
      z: 0.95,
      w: 0.3,
    },
  },
  detection_time: {
    sec: 162,
    nanosec: 500,
  },
  size: 0.15, // size of the detected ArUco marker
  dictionary_type: "DICT_4X4_50",
};

const ArucoSection: React.FC = () => {
  const [ArucoData, setArucoData] = useState<ArucoMsg | null>(null);

  const getArucoData = async () => {
    try {
      const res = await api.get<ArucoMsg>("http://localhost:8000/arucodata");
      setArucoData(res.data);
    } catch (err) {
      console.error("Failed to fetch Aruco data:", err);
      setArucoData(mockArucoData);
    }
  };

  useEffect(() => {
    getArucoData();
  }, []);

  const handleArucoData = () => {
    if (!ArucoData) {
      return (
        <div className="flex flex-col items-center justify-center py-6">
          <span className="text-gray-500 italic">No Pose Data Available</span>
        </div>
      );
    }

    const {
      pose: {
        position: { x: posX, y: posY, z: posZ },
        orientation: { x: oriX, y: oriY, z: oriZ, w: oriW },
      },
      detection_time,
      size,
      dictionary_type,
    } = ArucoData;

    return (
      <div className="grow w-full h-full  px-4 py-4 border-2 border-black rounded-xl shadow-md">
        <h2 className="text-2xl font-semibold text-navy mb-4 text-center">
          ArUco Marker Details
        </h2>
        <div className="flex flex-col md:flex-row justify-between gap-6">
          <div className="flex-1">
            <h3 className="text-lg font-medium text-gray-700 mb-2">Position</h3>
            <ul className="flex flex-row gap-x-6 text-red-700 font-mono text-base">
              <li>
                <span className="font-semibold">X:</span> {posX.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">Y:</span> {posY.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">Z:</span> {posZ.toFixed(3)}
              </li>
            </ul>
          </div>
          <div className="flex-1">
            <h3 className="text-lg font-medium text-gray-700 mb-2">
              Orientation
            </h3>
            <ul className="flex flex-row gap-x-4 text-blue-700 font-mono text-base">
              <li>
                <span className="font-semibold">X:</span> {oriX.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">Y:</span> {oriY.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">Z:</span> {oriZ.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">W:</span> {oriW.toFixed(3)}
              </li>
            </ul>
          </div>
        </div>
        <div className="flex flex-col md:flex-row justify-between gap-6 mt-4">
          <div className="flex-1">
            <h3 className="text-lg font-medium text-gray-700 mb-2">
              Detection Time
            </h3>
            <ul className="flex flex-row gap-x-4 text-green-700 font-mono text-base">
              <li>
                <span className="font-semibold">Sec:</span>{" "}
                {detection_time.sec.toFixed(3)}
              </li>
              <li>
                <span className="font-semibold">nSec:</span>{" "}
                {Math.round(detection_time.nanosec)}
              </li>
            </ul>
          </div>
          <div className="flex-1 flex flex-col gap-2">
            <div>
              <span className="font-semibold text-gray-700">Size:</span>{" "}
              <span className="text-purple-700 font-mono">{size} m</span>
            </div>
            <div>
              <span className="font-semibold text-gray-700">Dictionary:</span>{" "}
              <span className="text-orange-700 font-mono">
                {dictionary_type}
              </span>
            </div>
          </div>
        </div>
      </div>
    );
  };

  return <div>{handleArucoData()}</div>;
};

const CameraView: React.FC = () => {
  return (
    <div className="border-gray-200 rounded-2xl w-full h-full bg-red-500 flex flex-col">
      <div className="h-9/10 w-full border border-black rounded-2xl mx-2 my-1">
        Camera Feed
      </div>
      <h1 className="text-2xl text-center my-3  ">Pan and Tilt Feed</h1>
    </div>
  );
};
const PanTiltData: React.FC = () => {
  const [status, setStatus] = useState("TRACK");
  return (
    <div className="flex  flex-row border rounded-2xl bg-cosmic-latte w-1/2 h-175">
      <div className="flex flex-row grow ">
        <div className="flex flex-row  border-2 border-black rounded-xl shadow-md  w-full items-start justify-center px-3 py-5">
          <h1 className="text-xl font-bold text-navy">
            Pan and Tilt Mode:{" "}
            <span className="font-extrabold text-green-800">{status}</span>
          </h1>
        </div>
        <ArucoSection />
      </div>
    </div>
    // Show Status of Pan and TIlt
  );
};
function App() {
  return (
    <>
      <section className="flex flex-col h-dvh font-future bg-rblack w-full py-10 px-10">
        <h1 className="text-cosmic-latte text-5xl mb-3 text-center">QKD GUI</h1>
        {/* Pan And Tilt Section */}
        <div className="flex flex-row my-3 space-y-20  justify-between gap-x-10">
          <CameraView />
          <PanTiltData />
        </div>
      </section>
    </>
  );
}

export default App;
