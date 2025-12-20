# Dimos Mobile App

React Native mobile application (v0.0.1).

## Prerequisites

- **Node.js** >= 18.0.0
- **React Native CLI**: `npm install -g @react-native-community/cli`
- **Xcode** (for iOS development)
- **Android Studio** (for Android development)
- **CocoaPods** (for iOS): `sudo gem install cocoapods`

## Installation

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Install iOS dependencies** (macOS only):
   ```bash
   cd ios && pod install && cd ..
   ```

## Running the App

### Start Metro Bundler
```bash
npm start
```

### iOS
```bash
npm run ios
```

### Android
```bash
npm run android
```

## Development Scripts

- `npm start` - Start Metro bundler
- `npm run ios` - Run on iOS
- `npm run android` - Run on Android
- `npm run lint` - Run ESLint
- `npm test` - Run Jest tests

## Troubleshooting

**Clear Metro cache:**
```bash
npx react-native start --reset-cache
```

**iOS build issues:**
```bash
cd ios && rm -rf Pods && pod install && cd ..
```

**Android build issues:**
```bash
cd android && ./gradlew clean && cd ..
```

**Clear all caches:**
```bash
npx react-native start --reset-cache
rm -rf node_modules && npm install
```