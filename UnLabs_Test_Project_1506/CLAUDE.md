# CLAUDE.md — Unlabs AI Toolkit for FRC (WPILib 2026.2.1)

## Environment Settings
- Configuration: Secret tokens are stored locally in `.env` (ignored by git).
- Local Environment Override: `CLAUDE_CODE_AUTO_MODE_SERVER=0`

## Build and Test Commands
- Startup Command: `export $(cat .env | xargs) && claude --model qwen/qwen-2.5-coder-32b:free`
- Build project: `./gradlew build`
- Run local desktop simulation: `./gradlew simulateJava`
- Deploy code to roboRIO: `./gradlew deploy`

## FRC Code Style and Guardrails
- Language: Strictly Java, following the standard WPILib Command-Based framework.
- Package Paths: Use `edu.wpi.first.wpilibj2.command` structures.
- Hardware Verification: All motor definitions (SparkMax, TalonFX, KrakenX60) must include safety limits, inversion states, and current limits in their constructors.
- Interface Language: **Strictly respond and print terminal text in English.** Never output Chinese or other languages in the terminal or chat panel, even during inner thoughts or tool steps.