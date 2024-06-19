# Ubuntu 22.04 설치

Published by 이재우

Last update date : 2024-06-20



### Ubuntu version 22.04.04 LTS (Jammy Jellyfish) 를 설치합니다.
- desktop image 다운로드
  
  https://releases.ubuntu.com/22.04/?_gl=1*ziw8nv*_gcl_au*MTUyMjc2MTAzMi4xNzE4ODMxMzM5&_ga=2.267751881.653649382.1718831351-1186529121.1718831351

- 설치 방법이 정리된 블로그

  https://junorionblog.co.kr/ubuntu-22-04-desktop-%EC%84%A4%EC%B9%98-%EA%B0%80%EC%9D%B4%EB%93%9C/#google_vignette

### [ Note ]
1. WSL(Windows Subsystem for Linux)을 사용하면 QGroundControl 에서 오류가 발생하는 것을 확인하였습니다.(2024-06-20)

   Ubuntu Desktop 환경을 이용해야 합니다.
   
2. 실행 시 항상 패키지 update와 upgrade를 진행합니다.

   ```bash
   sudo apt update
   sudo apt upgrade
   ```


