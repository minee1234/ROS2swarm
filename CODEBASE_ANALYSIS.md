# ROS2swarm 코드베이스 전문가 분석 보고서

## 1. 프로젝트 개요

**ROS2swarm**은 독일 Konstanz 대학교 Cyber-Physical Systems 그룹에서 개발한 ROS 2 기반 다중 로봇 군집(swarm) 행동 패턴 프레임워크입니다.

| 항목 | 내용 |
|------|------|
| 라이선스 | Apache 2.0 |
| ROS 버전 | ROS 2 (Foxy/Galactic 호환) |
| 주요 언어 | Python (100%) |
| 패키지 수 | 3개 |
| 소스 파일 | 80개 Python 파일 |
| 코드 라인 | ~4,084 LOC |

---

## 2. 아키텍처 분석

### 2.1 패키지 구조

```
src/
├── ros2swarm/              # 핵심 행동 패턴 패키지 (ament_python)
├── communication_interfaces/ # 커스텀 메시지 정의 (ament_cmake)
└── launch_gazebo/          # Gazebo 시뮬레이션 인프라 (ament_python)
```

### 2.2 계층적 노드 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    Pattern Layer (행동 패턴)                 │
│  ┌──────────────────┐  ┌──────────────────┐                 │
│  │ MovementPattern  │  │  VotingPattern   │                 │
│  │ (drive_command)  │  │ (voting_broadcast)│                │
│  └────────┬─────────┘  └──────────────────┘                 │
│           │                                                  │
│           ▼                                                  │
│  ┌───────────────────────────────────────┐                  │
│  │     HardwareProtectionLayer           │                  │
│  │  (drive_command → cmd_vel 필터링)      │                  │
│  └────────┬──────────────────────────────┘                  │
│           │                                                  │
│           ▼                                                  │
│  ┌───────────────────────────────────────┐                  │
│  │         Sensor Layer                  │                  │
│  │  (LaserScan/Range → RangeData)        │                  │
│  └───────────────────────────────────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 클래스 상속 구조

```python
rclpy.node.Node
    │
    ├── AbstractPattern           # abstract_pattern.py:19
    │   │  - swarm_command 구독
    │   │  - start/stop 플래그 제어
    │   │  - swarm_command_controlled 데코레이터
    │   │
    │   ├── MovementPattern       # movement_pattern.py:18
    │   │   │  - drive_command 퍼블리셔
    │   │   │  - 정지 시 Twist() 발행
    │   │   │
    │   │   ├── DrivePattern
    │   │   ├── DispersionPattern
    │   │   ├── AggregationPattern
    │   │   ├── AttractionPattern (2종)
    │   │   ├── MagnetometerPattern
    │   │   ├── RandomWalkPattern
    │   │   ├── MinimalistFlockingPattern
    │   │   └── RatSearchPattern
    │   │
    │   ├── VotingPattern         # voting_pattern.py
    │   │   │  - 로봇 ID 추출
    │   │   │
    │   │   ├── VoterModelPattern
    │   │   ├── VoterModelWithLimiterPattern
    │   │   └── MajorityRulePattern
    │   │
    │   └── HardwareProtectionLayer
    │
    └── LidarLayer / IRLayer      # 센서 추상화 (AbstractPattern 상속 안함)
```

---

## 3. 핵심 컴포넌트 상세 분석

### 3.1 AbstractPattern (추상 기본 클래스)

**파일**: `abstract_pattern.py:19-78`

**설계 패턴**: Template Method Pattern

```python
class AbstractPattern(Node):
    def swarm_command_controlled(self, callback_func):
        """콜백을 start_flag로 제어하는 데코레이터 패턴"""
        def callb(x):
            if self.start_flag:
                callback_func(x)
            else:
                self.swarm_command_false_case()  # Hook method
        return lambda x: callb(x)
```

**장점**:
- 전역 `/swarm_command` 토픽으로 군집 전체 동기화 제어
- 데코레이터 패턴으로 깔끔한 콜백 래핑
- `swarm_command_false_case()`로 하위 클래스 확장 가능

**개선점**:
- `swarm_command_controlled`와 `swarm_command_controlled_timer`의 중복 코드 → 제네릭 데코레이터로 통합 가능

### 3.2 HardwareProtectionLayer (안전 계층)

**파일**: `hardware_protection_layer.py:24-152`

**역할**: 충돌 방지를 위한 명령 필터링

```
drive_command (패턴) → [HPL] → cmd_vel (로봇)
                         │
                    range_data (센서)
```

**핵심 로직** (`vector_calc` 메서드):
1. `ScanCalculationFunctions.potential_field()` 호출
2. 장애물 감지 시 회피 벡터 계산
3. 원본 명령을 회피 명령으로 대체

**설계 품질**:
- ✅ 센서 데이터와 명령 처리 분리
- ✅ 파라미터화된 임계값 (`threshold`, `max_range`)
- ⚠️ `self.current_angles`가 선언되었으나 사용 안 됨 (dead code)
- ⚠️ `angles` 속성이 `self.angles`로 직접 할당 (일관성 부족)

### 3.3 ScanCalculationFunctions (센서 처리 유틸리티)

**파일**: `scan_calculation_functions.py:47-565`

**핵심 알고리즘**:

| 메서드 | 용도 |
|--------|------|
| `potential_field()` | 반발력 기반 회피 벡터 계산 |
| `attraction_field()` | 인력 기반 접근 벡터 계산 |
| `identify_objects()` | 레이저 스캔에서 객체 클러스터링 |
| `identify_robots()` | 객체 중 로봇 식별 (폭 기반 필터링) |

**수학적 기반**:
```python
def linear_rating(ranges, max_range):
    """선형 거리 가중치: 가까울수록 높은 가중치"""
    return [1 - (x / max_range) for x in ranges]

def create_normed_twist_message(vector, max_trans, max_rot):
    """정규화된 속도 명령 생성"""
    direction.linear.x = max_trans * normalized_vector[0]
    direction.angular.z = max_rot * asin(normalized_vector[1] / hypot(...))
```

**코드 품질**:
- ✅ 순수 함수 (`@staticmethod`) 사용으로 테스트 용이
- ✅ NaN 방어 코드 (`np.isnan` 체크)
- ⚠️ 565줄의 대형 클래스 → 기능별 모듈 분리 권장
- ⚠️ `ReductionOption` enum 일부만 구현 (`FARTHEST` 미구현 in `reduce_objects`)

### 3.4 VoterModelPattern (합의 알고리즘)

**파일**: `voter_model_pattern.py:25-120`

**알고리즘**: Voter Model (확률적 합의)

```python
def timer_callback(self):
    if len(self.opinion_list) > 0:
        chosen_opinion = random.choice(self.opinion_list)  # 무작위 선택
        self.opinion = chosen_opinion.opinion
    self.broadcast_publisher.publish(self.opinion_message)
```

**통신 패턴**:
- 글로벌 `/voting_broadcast` 토픽 (전체 로봇 공유)
- `OpinionMessage`: `{id: int64, opinion: int64}`

**설계 분석**:
- ✅ 분산 합의 알고리즘의 정확한 구현
- ✅ 초기값 랜덤/고정 선택 옵션
- ⚠️ `first_broadcast_flag` 로직이 첫 방송 전 의견 수집 방지 (의도적)

---

## 4. 커스텀 메시지 인터페이스

**패키지**: `communication_interfaces`

| 메시지 | 필드 | 용도 |
|--------|------|------|
| `RangeData` | `header`, `ranges[]`, `angles[]` | 센서 추상화 |
| `OpinionMessage` | `id`, `opinion` | 투표 패턴 |
| `OpinionMACMessage` | `id`, `opinion`, `mac` | WiFi 기반 투표 |
| `ModelStatesStamped` | `header`, `name[]`, `pose[]`, `twist[]` | Gazebo 상태 |

**설계 원칙**:
- 센서 비의존적 인터페이스 (`RangeData`가 LiDAR/IR 모두 지원)
- 타임스탬프가 있는 메시지 (`header` 포함)

---

## 5. 파라미터 시스템 분석

### 5.1 구성 체계

```
config/
├── {robot_type}/
│   ├── hardware_protection_layer.yaml
│   ├── sensor_specification.yaml
│   ├── movement_pattern/
│   │   ├── basic/{pattern}.yaml
│   │   └── combined/{pattern}.yaml
│   └── voting_pattern/
│       └── basic/{pattern}.yaml
```

### 5.2 파라미터 예시 (TurtleBot3 Burger)

```yaml
# hardware_protection_layer.yaml
hardware_protection_layer_max_range: 0.300   # 300mm = 반경 + 정확도 + 버퍼
hardware_protection_layer_min_range: 0.12    # 센서 최소 거리
hardware_protection_layer_threshold: 5       # 장애물 감지 임계 레이 수
max_translational_velocity: 0.22             # TurtleBot3 최대 속도
max_rotational_velocity: 2.84                # TurtleBot3 최대 각속도
```

**로봇별 튜닝**:
- `burger`: 소형, 낮은 속도 (0.22 m/s)
- `waffle_pi`: 중형, 같은 속도
- `jackal`: 대형, 더 넓은 감지 범위
- `thymio`: IR 센서용 특수 설정

---

## 6. 런치 시스템 분석

### 6.1 메인 런치 파일 (`bringup_patterns.launch.py`)

**실행 흐름**:
```
bringup_patterns.launch.py
  │
  ├── [1] sensor_layer (lidar_layer / ir_layer)
  │       → sensor_specification.yaml
  │
  ├── [2] hardware_protection_layer
  │       → hardware_protection_layer.yaml
  │
  ├── [3] pattern (선택된 행동 패턴)
  │       → pattern별 yaml
  │
  └── [4] robot_state_publisher (조건부)
          └── [4.1] nav2 localization (driving_swarm=True인 경우)
```

**파라미터화**:
```python
config_dir = LaunchConfiguration('config_dir')
pattern = LaunchConfiguration('pattern')        # 패턴 런치 파일 경로
sensor_type = LaunchConfiguration('sensor_type') # 'lidar' 또는 'ir'
robot_type = LaunchConfiguration('robot_type')  # 'burger', 'jackal', etc.
```

### 6.2 동적 executable 구성

```python
# 센서 타입에 따른 동적 노드 선택
executable=[sensor_type, '_layer']  # 'lidar_layer' 또는 'ir_layer'
```

---

## 7. 테스트 인프라 분석

**테스트 디렉토리**: `src/ros2swarm/test/`

| 테스트 파일 | 라인 수 | 설명 |
|-------------|---------|------|
| `test_scan_calculation_functions.py` | 16,711 | 센서 처리 단위 테스트 |
| `test_vote_list.py` | 4,097 | 투표 리스트 테스트 |
| `test_drive_pattern.py` | ~50 | 드라이브 패턴 통합 테스트 |
| `test_flake8.py` | - | PEP 8 스타일 검사 |
| `test_pep257.py` | - | Docstring 검사 |
| `test_copyright.py` | - | 저작권 헤더 검사 |

**테스트 커버리지 분석**:
- ✅ `ScanCalculationFunctions`: 매우 포괄적 (16K LOC)
- ⚠️ 패턴 노드 테스트: 최소한의 통합 테스트만 존재
- ❌ `HardwareProtectionLayer`: 전용 테스트 없음
- ❌ 센서 레이어: 전용 테스트 없음

---

## 8. 지원 로봇 플랫폼

| 플랫폼 | 센서 | 특징 |
|--------|------|------|
| TurtleBot3 Burger | LiDAR | 소형, 교육용, 실내 |
| TurtleBot3 Waffle Pi | LiDAR | 중형, 카메라 포함 |
| Clearpath Jackal | LiDAR | 대형, 야외 가능 |
| Thymio II | IR | 교육용, 단순 센서 |
| LIMO | LiDAR | 다목적 |

---

## 9. 코드 품질 평가

### 9.1 강점

| 영역 | 평가 |
|------|------|
| **모듈화** | ⭐⭐⭐⭐⭐ 패턴/센서/보호 계층 명확 분리 |
| **확장성** | ⭐⭐⭐⭐ 새 패턴/로봇 추가 용이 |
| **문서화** | ⭐⭐⭐⭐ 명확한 docstring |
| **설정 관리** | ⭐⭐⭐⭐⭐ 로봇별 YAML 분리 |
| **ROS 2 준수** | ⭐⭐⭐⭐ 표준 패턴 사용 |

### 9.2 개선 필요 영역

| 영역 | 문제점 | 권장 사항 |
|------|--------|-----------|
| **Dead Code** | `current_angles` 미사용 (hardware_protection_layer.py:56) | 제거 |
| **타입 힌트** | 일부 함수 누락 | 전체 타입 힌트 추가 |
| **에러 처리** | 센서 데이터 None 체크 최소 | 더 robust한 예외 처리 |
| **테스트 커버리지** | HPL, 센서 레이어 테스트 부재 | 통합 테스트 확대 |
| **setup.py 오타** | 66, 76행 `'thymio'`, `'limo'` 잘못된 경로 | `'share'`로 수정 필요 |

### 9.3 setup.py 버그

```python
# 66행 - 오류
(os.path.join('thymio', package_name, ...),  # 'share' 누락

# 76행 - 오류
(os.path.join('limo', package_name, ...),    # 'share' 누락
```

---

## 10. 아키텍처 다이어그램

### 10.1 토픽 흐름도

```
                    /swarm_command (Int8Message)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
  ┌───────────┐     ┌───────────┐     ┌───────────┐
  │ Pattern 1 │     │ Pattern 2 │     │ Pattern N │
  └─────┬─────┘     └─────┬─────┘     └─────┬─────┘
        │                 │                 │
        ▼                 ▼                 ▼
  /{ns}/drive_command   /{ns}/drive_command  ...
        │                 │
        └────────┬────────┘
                 ▼
        ┌─────────────────────┐
        │ HardwareProtection  │ ←── /{ns}/range_data
        │      Layer          │
        └─────────┬───────────┘
                  ▼
           /{ns}/cmd_vel
                  │
                  ▼
            [Robot Base]
```

### 10.2 투표 패턴 통신

```
     Robot 1              Robot 2              Robot 3
        │                    │                    │
        └─────── /voting_broadcast ───────────────┘
                 (전역 브로드캐스트)
```

---

## 11. 행동 패턴 상세

### 11.1 이동 패턴 (Movement Patterns)

#### Basic Patterns (9개)

| 패턴 | 파일 | 설명 |
|------|------|------|
| `drive_pattern` | `drive_pattern.py` | 일정 속도로 직진/회전 |
| `dispersion_pattern` | `dispersion_pattern.py` | 로봇 간 거리 유지 (분산) |
| `aggregation_pattern` | `aggregation_pattern.py` | 로봇들 모이기 (집합) |
| `attraction_pattern` | `attraction_pattern.py` | 특정 지점으로 이동 |
| `attraction_pattern2` | `attraction_pattern2.py` | 대안적 인력 구현 |
| `magnetometer_pattern` | `magnetometer_pattern.py` | 나침반 기반 방향 제어 |
| `random_walk_pattern` | `random_walk_pattern.py` | 무작위 탐색 |
| `minimalist_flocking_pattern` | `minimalist_flocking_pattern.py` | 군집 비행 행동 |
| `rat_search_pattern` | `rat_search_pattern.py` | 정보 기반 탐색 |

#### Combined Patterns (1개)

| 패턴 | 파일 | 설명 |
|------|------|------|
| `discussed_dispersion_pattern` | `discussed_dispersion_pattern.py` | 기본 패턴 조합 |

### 11.2 투표 패턴 (Voting Patterns)

| 패턴 | 파일 | 알고리즘 |
|------|------|----------|
| `voter_model_pattern` | `voter_model_pattern.py` | 무작위 의견 채택 |
| `voter_model_with_limiter_pattern` | `voter_model_with_limiter_pattern.py` | WiFi 신호 강도 기반 제한 |
| `majority_rule_pattern` | `majority_rule_pattern.py` | 다수결 합의 |

---

## 12. 시뮬레이션 환경

### 12.1 Gazebo 월드 파일

| 파일 | 크기 | 용도 |
|------|------|------|
| `empty.world` | 소형 | 기본 빈 환경 |
| `arena.world` | 중형 | 오픈 아레나 |
| `arena_large.world` | 대형 | 대규모 테스트 |
| `Ymaze.world` | 450MB+ | Y-미로 실험 |

### 12.2 시뮬레이션 유틸리티

- `add_bot_node.py`: 동적 로봇 스폰
- `ground_truth_publisher.py`: Gazebo 상태 발행

---

## 13. 결론

**ROS2swarm**은 학술 연구 목적에 적합한 잘 설계된 군집 로봇 프레임워크입니다.

### 핵심 강점

1. 계층적 아키텍처로 관심사 분리 우수
2. 센서 추상화 (`RangeData`)로 다양한 로봇 지원
3. 파라미터 기반 설정으로 재사용성 높음
4. Template Method 패턴으로 확장 용이

### 실무 적용 시 고려사항

1. 테스트 커버리지 확대 필요
2. setup.py 버그 수정 필요
3. 대규모 군집에서의 `/voting_broadcast` 스케일링 검토 필요

### 참고 문헌

ICRA 2022 논문 (README.md 참조)

---

*분석 일자: 2026-02-22*
*분석 도구: Claude Code*
