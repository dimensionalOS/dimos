import React from 'react';
import {
  Alert,
  SafeAreaView,
  StatusBar,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from 'react-native';
import FigletText from '../utils/FigletText';

interface HomeScreenProps {
  onNavigateToSettings: () => void;
}

const HomeScreen: React.FC<HomeScreenProps> = ({onNavigateToSettings}) => {
  const handleAddRobotDog = () => {
    Alert.alert('Add Robot', 'This feature will be implemented soon!');
  };

  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" />
      <View style={styles.headerRow}>
        <FigletText text="DIMENSIONAL" color="#FFF200" fontSize={4} />
        <TouchableOpacity 
          style={styles.gearButton} 
          onPress={onNavigateToSettings}
          activeOpacity={0.7}
        >
          <View style={styles.settingsIcon}>
            <Text style={styles.gearText}>⋯</Text>
          </View>
        </TouchableOpacity>
      </View>
      <View style={styles.centerArea}>
        <View style={styles.ctaContainer}>
          <TouchableOpacity
            style={styles.cta}
            activeOpacity={0.9}
            onPress={handleAddRobotDog}>
            <FigletText text="ADD ROBOT" color="#0016B1" fontSize={5}/>
          </TouchableOpacity>
        </View>
      </View>
      <View style={styles.footer}>
        <Text style={styles.version}>v0.0.1</Text>
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#0016B1',
  },
  headerRow: {
    paddingTop: 24,
    paddingHorizontal: 24,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  gearButton: {
    backgroundColor: '#FFF200',
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  settingsIcon: {
    alignItems: 'center',
    justifyContent: 'center',
  },
  gearText: {
    fontSize: 24,
    color: '#0016B1',
    fontWeight: 'bold',
  },
  centerArea: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 24,
  },
  ctaContainer: {
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: -40,
  },
  cta: {
    backgroundColor: '#FFF200',
    paddingHorizontal: 20,
    paddingVertical: 12,
    borderRadius: 16,
    shadowColor: '#000',
    shadowOpacity: 0.25,
    shadowOffset: {width: 0, height: 2},
    shadowRadius: 4,
    elevation: 3,
    alignItems: 'center',
    justifyContent: 'center',
  },
  footer: {
    alignItems: 'center',
    paddingBottom: 32,
    paddingTop: 16,
  },
  version: {
    color: '#FFF200',
    fontSize: 12,
    fontFamily: 'monospace',
  },
});

export default HomeScreen;
