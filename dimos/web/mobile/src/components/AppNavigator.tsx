import React, {useState, useEffect} from 'react';
import LoadingScreen from '../screens/LoadingScreen';
import HomeScreen from '../screens/HomeScreen';
import SettingsScreen from '../screens/SettingsScreen';

type Screen = 'loading' | 'home' | 'settings';

const AppNavigator: React.FC = () => {
  const [currentScreen, setCurrentScreen] = useState<Screen>('loading');

  useEffect(() => {
    const timer = setTimeout(() => setCurrentScreen('home'), 1200);
    return () => clearTimeout(timer);
  }, []);
  if (currentScreen === 'loading') {
    return <LoadingScreen />;
  }

  const navigateToSettings = () => {
    setCurrentScreen('settings');
  };

  const navigateToHome = () => {
    setCurrentScreen('home');
  };

  if (currentScreen === 'settings') {
    return <SettingsScreen onBack={navigateToHome} />;
  }

  return <HomeScreen onNavigateToSettings={navigateToSettings} />;
};

export default AppNavigator;

