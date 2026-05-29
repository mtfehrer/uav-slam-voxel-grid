Record:
docker-compose up -f docker-compose.record.yml
in container:
./record-init.sh
./start-recording.sh

Playback:
docker-compose up -f docker-compose.playback.yml
in container:
./playback-init.sh