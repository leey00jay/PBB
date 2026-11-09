# 테스트

`test_*.py` 파일에 `unittest.TestCase`를 작성합니다. 새 하위 폴더에는 `__init__.py`를 추가해야
Python 3.10/3.12의 재귀 검색에 포함됩니다.

저장소 최상위 폴더에서 `src`를 Python 경로에 포함해 실행합니다. GitHub Actions도 같은 환경을 설정합니다.

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

실제 알고리즘 추가 시 필요한 테스트 의존성도 CI 설치 단계에 함께 반영합니다.
