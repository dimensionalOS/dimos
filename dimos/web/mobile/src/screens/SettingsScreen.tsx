import React from 'react';
import {
  SafeAreaView,
  StatusBar,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
  ScrollView,
  Alert,
} from 'react-native';
import FigletText from '../utils/FigletText';

interface SettingsScreenProps {
  onBack: () => void;
}

const SettingsScreen: React.FC<SettingsScreenProps> = ({onBack}) => {
  const handleAbout = () => {
    Alert.alert(
      'About',
      'Powering generalist robotics',
      [{text: 'OK'}]
    );
  };

  const handleTermsAndConditions = () => {
    Alert.alert(
      'Terms & Conditions',
      'By using this app, you agree to our terms of service. This app is provided as-is for educational and entertainment purposes.',
      [{text: 'OK'}]
    );
  };

  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" />
      
      {/* Header */}
      <View style={styles.header}>
        <TouchableOpacity 
          style={styles.backButton} 
          onPress={onBack}
          activeOpacity={0.7}
        >
          <FigletText text="BACK" color="#0016B1" fontSize={2} />
        </TouchableOpacity>
        <FigletText text="SETTINGS" color="#FFF200" fontSize={4} />
        <View style={styles.placeholder} />
      </View>

      {/* Settings Content */}
      <ScrollView style={styles.content} showsVerticalScrollIndicator={false}>
        <View style={styles.section}>
          <TouchableOpacity 
            style={styles.settingItem} 
            onPress={handleAbout}
            activeOpacity={0.7}
          >
            <View style={styles.settingItemContent}>
              <FigletText text="ABOUT" color="#FFF200" fontSize={2} />
              <FigletText text=">" color="#FFF200" fontSize={3} />
            </View>
          </TouchableOpacity>

          <TouchableOpacity 
            style={styles.settingItem} 
            onPress={handleTermsAndConditions}
            activeOpacity={0.7}
          >
            <View style={styles.settingItemContent}>
              <FigletText text="TERMS & CONDITIONS" color="#FFF200" fontSize={2} />
              <FigletText text=">" color="#FFF200" fontSize={3} />
            </View>
          </TouchableOpacity>
        </View>
      </ScrollView>

      {/* Footer */}
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
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingTop: 24,
    paddingHorizontal: 24,
    paddingBottom: 16,
  },
  backButton: {
    backgroundColor: '#FFF200',
    paddingHorizontal: 6,
    paddingVertical: 4,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  placeholder: {
    width: 36,
  },
  content: {
    flex: 1,
    paddingHorizontal: 24,
  },
  section: {
    marginTop: 24,
  },
  settingItem: {
    backgroundColor: 'rgba(255, 242, 0, 0.1)',
    borderRadius: 12,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: 'rgba(255, 242, 0, 0.2)',
  },
  settingItemContent: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 12,
    paddingVertical: 12,
  },
  footer: {
    alignItems: 'center',
    paddingBottom: 16,
    paddingTop: 24,
  },
  version: {
    color: '#FFF200',
    fontSize: 12,
    fontFamily: 'monospace',
    opacity: 0.7,
  },
});

export default SettingsScreen;
