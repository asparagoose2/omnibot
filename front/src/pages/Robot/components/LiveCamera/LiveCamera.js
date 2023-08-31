import settings from '../../../../settings';
import './live-camera.css';

const {url: liveCameraUrl} = settings.liveCamera;

const LiveCamera = () => (
    <img className='live-camera' alt='live camera' src={`${liveCameraUrl}`} />
);

export default LiveCamera;
