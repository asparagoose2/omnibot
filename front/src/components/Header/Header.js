import avatar from '../../images/avatar.jpg';
import { Controller } from '../../pages/Robot/components';
import './header.css';

function Header() {
    return (
        <div className="header">
            OmniBot
            <img className="avatar" src={avatar} />
        </div>
    );
}

export default Header;
