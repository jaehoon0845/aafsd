# ─── Build stage ──────────────────────────────────────────────
# Eclipse Temurin JDK 21 + 프로젝트의 gradle wrapper (gradle 9.4.1) 사용
# → Render 와 로컬에서 동일한 gradle 버전 보장
FROM eclipse-temurin:21-jdk AS build
WORKDIR /app

# Gradle wrapper 먼저 복사 (의존성 다운로드 캐시 최적화)
COPY gradlew gradlew
COPY gradle gradle
RUN chmod +x gradlew

# Build 정의 파일 복사 후 의존성만 미리 다운로드
COPY build.gradle settings.gradle ./
RUN ./gradlew dependencies --no-daemon || true

# 소스 복사 후 JAR 빌드 (테스트 제외)
COPY src ./src
RUN ./gradlew bootJar -x test --no-daemon

# ─── Runtime stage ────────────────────────────────────────────
# 실행에는 JRE 만 필요 (이미지 크기 최소화)
FROM eclipse-temurin:21-jre
WORKDIR /app

# 와일드카드로 JAR 복사 (version 바뀌어도 깨지지 않음)
COPY --from=build /app/build/libs/*.jar app.jar

# Render 가 PORT 환경변수를 주입함 → application.properties 의 ${PORT:8080} 으로 읽음
EXPOSE 8080
CMD ["java", "-jar", "app.jar"]
