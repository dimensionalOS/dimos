import React from 'react';
import {SafeAreaView, StatusBar, StyleSheet, View} from 'react-native';
import FigletText from '../utils/FigletText';

const LoadingScreen: React.FC = () => {
  return (
    <SafeAreaView style={styles.container}>
      <StatusBar barStyle="light-content" />
      <View style={styles.logoRow}>
        <FigletText text="DIMENSIONAL" color="#FFF200" fontSize={4} />
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#0016B1',
    justifyContent: 'center',
  },
  logoRow: {
    paddingHorizontal: 24,
    alignItems: 'center',
  },
  logoImage: {
    width: 120,
    height: 120,
    marginBottom: 16,
  },
  appName: {
    color: '#FFF200',
    fontSize: 32,
    fontWeight: '800',
    letterSpacing: 2,
    fontFamily: 'monospace',
    marginBottom: 8,
  },
  version: {
    color: '#FFF200',
    fontSize: 12,
    marginTop: 4,
    fontFamily: 'monospace',
  },
});

export default LoadingScreen;
