# Git 3회차 live 세션

1. 협업할 때 중요한 요소
    1) 코딩 스타일 공유 및 통일
    + 변수명, 인덴트 등
    2) 일을 분배하는 기준, 역할 분담 
    + 팀원 각자의 강점 파악, 제한된 시간에 따른 우선순위 파악
    3) 의사 소통 규칙
    + 회의 시간, 작업 상황 공유, 
    4) Workflow
    + Repository 운영 / Branching 전략 / Issue, PR 관리

2. 코딩 스타일
* PEP 8 : 권장 코드 표기법
* formatter 
    + black ** (PEP 8 기준으로 맞춰줌)
    + yapf (custom 설정이 가능)
    + isort (import에 특화됨)
* Linter
    + pylint 
    + flake8

3. Workflow - Repository 운영 방법
* Centralized
    + 관리 포인트가 하나
    + 충돌이 났을 경우에 팀원들에게 바로 영향을 미침, 위험성 존재

* Forking
    + 관리 포인트가 늘어남
    + 좀 더 도전적인 시도를 해볼 수 있음

4. Workflow - Branching 관리 전략

5. Workflow - Issue 관리
* 문제, 할 일, 관심사
* Issue Labels : 우선순위, 상태, 타입 등

6. Workflow - Pull Request
* 코드를 통합하는 방법 중 하나 
* 코드 리뷰 수행
* 오픈 소스 기여

7. Workflow - Conflict
* 코드 스타일, 파일 분리, branch 분리, 


8. 기타 꿀팁
* .gitignore에 setting 등 설정
* pip freeze > requirements. txt -> 버전txt
* 템플릿 설정(issue, PR 템플릿 등)


> 궁금한사항 : fast-forward 전략 ? rebase 전략? pre-commit? git-hooks? Github Actions?