import { useEffect, useState, useRef } from 'react';
import { Header } from './components/common/Header';
import { Notifications } from './components/common/Notifications';
import { StatusPanel } from './components/Dashboard/StatusPanel';
import { ControlPanel } from './components/Dashboard/ControlPanel';
import { MapCanvas } from './components/Map/MapCanvas';
import { useWebSocket } from './hooks/useWebSocket';
import { useRobotStore } from './store/robotStore';
import { mapApi } from './services/api';

function App() {
  useWebSocket();

  const { setActiveMap } = useRobotStore();
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const [mapSize, setMapSize] = useState({ width: 800, height: 600 });

  // Load active map on mount
  useEffect(() => {
    const loadActiveMap = async () => {
      try {
        const response = await mapApi.getActive();
        if (response.data.active && response.data.map) {
          setActiveMap(response.data.map);
        }
      } catch (error) {
        console.error('Failed to load active map:', error);
      }
    };
    loadActiveMap();
  }, [setActiveMap]);

  // Handle resize
  useEffect(() => {
    const updateSize = () => {
      if (mapContainerRef.current) {
        setMapSize({
          width: mapContainerRef.current.clientWidth,
          height: mapContainerRef.current.clientHeight,
        });
      }
    };

    updateSize();
    window.addEventListener('resize', updateSize);
    return () => window.removeEventListener('resize', updateSize);
  }, []);

  return (
    <div className="min-h-screen bg-gray-100 flex flex-col">
      <Header />

      <main className="flex-1 p-4 flex gap-4">
        {/* Map Area */}
        <div
          ref={mapContainerRef}
          className="flex-1 bg-white rounded-lg shadow overflow-hidden"
        >
          <MapCanvas width={mapSize.width} height={mapSize.height} />
        </div>

        {/* Sidebar */}
        <div className="w-80 space-y-4 flex-shrink-0">
          <StatusPanel />
          <ControlPanel />
        </div>
      </main>

      <Notifications />
    </div>
  );
}

export default App;
